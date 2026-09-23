//! SG2002 硬件 JPEG 编码（VENC / PT_JPEG）薄封装。
//!
//! 设计要点：
//! - **运行期 dlopen** `/mnt/system/usr/lib/{libsys.so,libvenc.so}`，不把厂商 .so 放进仓库，
//!   没有 SDK 的镜像上 `hwjpeg_init` 直接失败 → 上层降级到软件编码；
//! - **输入用相机原生的 YUYV422**（`PIXEL_FORMAT_YUYV`），免颜色转换，只需按 stride 逐行拷贝；
//! - 每帧：VB 公共池取块 → `CVI_SYS_MmapCache` → 拷 YUYV → `CVI_SYS_IonFlushCache`
//!   → `CVI_VENC_SendFrame` → `CVI_VENC_GetStream` → 拷出 JPEG → `ReleaseStream` → 归还块；
//! - 通道/线程约定：`hwjpeg_encode` 必须与 `hwjpeg_init` 在同一线程调用（VENC 通道无锁）。
//!
//! 编译（交叉工具链 + SDK 头文件）：
//!   riscv64-linux-musl-gcc -O2 -DHWJPEG_MAIN -I<sdk>/include -I<sdk>/include/linux
//!       -o hwjpeg_test csrc/hwjpeg.c -ldl
//!
//! 独立测试模式（`-DHWJPEG_MAIN`）：读一张 640x480 YUYV422 裸图，编码 N 帧并计时。

#include <dlfcn.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cvi_buffer.h"
#include "cvi_comm_vb.h"
#include "cvi_sys.h"
#include "cvi_vb.h"
#include "cvi_venc.h"

/* ---- 固定参数（与上位机管线一致）---- */
#define HWJPEG_WIDTH 640
#define HWJPEG_HEIGHT 480
#define HWJPEG_CHN 0
#define HWJPEG_VB_BLK_CNT 4
#define HWJPEG_MAX_PACK 8
#define HWJPEG_TIMEOUT_MS 2000
#define HWJPEG_LIB_SYS "/mnt/system/usr/lib/libsys.so"
#define HWJPEG_LIB_VENC "/mnt/system/usr/lib/libvenc.so"
#define HWJPEG_LIB_VENC_ALT "libvenc.so"

/* VENC 的 JPEG 通道实际只认 semi-planar YUV420：相机是 packed YUYV422，
 * 所以这里在写 VB 块时做一次 YUYV→NV12/NV21 转换（可用 -D 覆盖）。
 * 注：PIXEL_FORMAT_YUYV 会被驱动当成 NV12 解释（画面变成绿/品红条纹）。 */
#ifndef HWJPEG_INPUT_FORMAT
#define HWJPEG_INPUT_FORMAT PIXEL_FORMAT_NV12
#endif
#define HWJPEG_SWAP_UV (HWJPEG_INPUT_FORMAT == PIXEL_FORMAT_NV21)

/* ---- dlopen 出来的入口 ---- */
typedef CVI_S32 (*fn_sys_init)(void);
typedef CVI_S32 (*fn_sys_exit)(void);
typedef void *(*fn_sys_mmap)(CVI_U64, CVI_U32);
typedef void *(*fn_sys_mmap_cache)(CVI_U64, CVI_U32);
typedef CVI_S32 (*fn_sys_munmap)(void *, CVI_U32);
typedef CVI_S32 (*fn_sys_flush)(CVI_U64, void *, CVI_U32);
typedef CVI_S32 (*fn_vb_set_config)(const VB_CONFIG_S *);
typedef CVI_S32 (*fn_vb_init)(void);
typedef CVI_S32 (*fn_vb_exit)(void);
typedef VB_BLK (*fn_vb_get_block)(VB_POOL, CVI_U32);
typedef CVI_S32 (*fn_vb_release_block)(VB_BLK);
typedef CVI_U64 (*fn_vb_handle2phys)(VB_BLK);
typedef CVI_U32 (*fn_vb_handle2pool)(VB_BLK);
typedef CVI_S32 (*fn_venc_create)(VENC_CHN, const VENC_CHN_ATTR_S *);
typedef CVI_S32 (*fn_venc_destroy)(VENC_CHN);
typedef CVI_S32 (*fn_venc_set_jpeg)(VENC_CHN, const VENC_JPEG_PARAM_S *);
typedef CVI_S32 (*fn_venc_start)(VENC_CHN, const VENC_RECV_PIC_PARAM_S *);
typedef CVI_S32 (*fn_venc_stop)(VENC_CHN);
typedef CVI_S32 (*fn_venc_send)(VENC_CHN, const VIDEO_FRAME_INFO_S *, CVI_S32);
typedef CVI_S32 (*fn_venc_get_stream)(VENC_CHN, VENC_STREAM_S *, CVI_S32);
typedef CVI_S32 (*fn_venc_release_stream)(VENC_CHN, VENC_STREAM_S *);

static struct {
	int ready;
	char err[256];
	void *h_sys, *h_venc;
	fn_sys_init sys_init;
	fn_sys_exit sys_exit;
	fn_sys_mmap sys_mmap;
	fn_sys_mmap_cache sys_mmap_cache;
	fn_sys_munmap sys_munmap;
	fn_sys_flush sys_flush;
	fn_vb_set_config vb_set_config;
	fn_vb_init vb_init;
	fn_vb_exit vb_exit;
	fn_vb_get_block vb_get_block;
	fn_vb_release_block vb_release_block;
	fn_vb_handle2phys vb_handle2phys;
	fn_vb_handle2pool vb_handle2pool;
	fn_venc_create venc_create;
	fn_venc_destroy venc_destroy;
	fn_venc_set_jpeg venc_set_jpeg;
	fn_venc_start venc_start;
	fn_venc_stop venc_stop;
	fn_venc_send venc_send;
	fn_venc_get_stream venc_get_stream;
	fn_venc_release_stream venc_release_stream;
	/* 帧缓冲布局 */
	CVI_U32 vb_size;
	CVI_U32 stride;
	CVI_U32 y_size;
	CVI_U32 c_stride;
	CVI_U32 c_size;
	CVI_U16 addr_align;
	CVI_U8 plane_num;
	VENC_PACK_S packs[HWJPEG_MAX_PACK];
	CVI_U64 pts;
} g;

static void set_err(const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(g.err, sizeof(g.err), fmt, ap);
	va_end(ap);
}

const char *hwjpeg_error(void)
{
	return g.err[0] ? g.err : "ok";
}

/* 供上层查询：实际使用的输入像素格式。 */
uint32_t hwjpeg_input_format(void)
{
	return (uint32_t)HWJPEG_INPUT_FORMAT;
}

static void *load_sym(void *h, const char *name)
{
	void *p = dlsym(h, name);
	if (!p)
		set_err("dlsym %s 失败: %s", name, dlerror());
	return p;
}

#define LOAD(field, name)                          \
	do {                                       \
		g.field = (void *)load_sym(g.h_venc, name); \
		if (!g.field) return -1;           \
	} while (0)

#define LOAD_SYS(field, name)                      \
	do {                                       \
		g.field = (void *)load_sym(g.h_sys, name);  \
		if (!g.field) return -1;           \
	} while (0)

int hwjpeg_init(uint32_t width, uint32_t height, uint32_t quality)
{
	if (g.ready)
		return 0;
	if (width != HWJPEG_WIDTH || height != HWJPEG_HEIGHT) {
		set_err("只支持 %dx%d，收到 %ux%u", HWJPEG_WIDTH, HWJPEG_HEIGHT, width, height);
		return -1;
	}
	set_err("ok");

	/* libsys 里有 1 字节 CAS（C906 无原子指令），依赖 libatomic 提供
	 * __atomic_compare_exchange_1；musl 不会自动跨库解析，先把它预载进全局作用域。 */
	static const char *atomic_paths[] = {
		"/usr/lib/libatomic.so.1",
		"/lib/libatomic.so.1",
		"libatomic.so.1",
		"/mnt/system/usr/lib/libatomic.so.1",
	};
	for (size_t i = 0; i < sizeof(atomic_paths) / sizeof(atomic_paths[0]); i++) {
		void *h = dlopen(atomic_paths[i], RTLD_NOW | RTLD_GLOBAL);
		if (h)
			break;
	}

	/* 厂商库：libsys 先加载（RTLD_GLOBAL），libvenc 的 DT_NEEDED 才能解析到 */
	g.h_sys = dlopen(HWJPEG_LIB_SYS, RTLD_NOW | RTLD_GLOBAL);
	if (!g.h_sys) {
		set_err("dlopen %s 失败: %s（没有厂商 SDK 时属正常，可降级软件编码）",
			HWJPEG_LIB_SYS, dlerror());
		return -1;
	}
	g.h_venc = dlopen(HWJPEG_LIB_VENC, RTLD_NOW | RTLD_GLOBAL);
	if (!g.h_venc)
		g.h_venc = dlopen(HWJPEG_LIB_VENC_ALT, RTLD_NOW | RTLD_GLOBAL);
	if (!g.h_venc) {
		set_err("dlopen %s 失败: %s", HWJPEG_LIB_VENC, dlerror());
		return -1;
	}

	LOAD_SYS(sys_init, "CVI_SYS_Init");
	LOAD_SYS(sys_exit, "CVI_SYS_Exit");
	LOAD_SYS(sys_mmap, "CVI_SYS_Mmap");
	LOAD_SYS(sys_mmap_cache, "CVI_SYS_MmapCache");
	LOAD_SYS(sys_munmap, "CVI_SYS_Munmap");
	LOAD_SYS(sys_flush, "CVI_SYS_IonFlushCache");
	LOAD_SYS(vb_set_config, "CVI_VB_SetConfig");
	LOAD_SYS(vb_init, "CVI_VB_Init");
	LOAD_SYS(vb_exit, "CVI_VB_Exit");
	LOAD_SYS(vb_get_block, "CVI_VB_GetBlock");
	LOAD_SYS(vb_release_block, "CVI_VB_ReleaseBlock");
	LOAD_SYS(vb_handle2phys, "CVI_VB_Handle2PhysAddr");
	LOAD_SYS(vb_handle2pool, "CVI_VB_Handle2PoolId");
	LOAD(venc_create, "CVI_VENC_CreateChn");
	LOAD(venc_destroy, "CVI_VENC_DestroyChn");
	LOAD(venc_set_jpeg, "CVI_VENC_SetJpegParam");
	LOAD(venc_start, "CVI_VENC_StartRecvFrame");
	LOAD(venc_stop, "CVI_VENC_StopRecvFrame");
	LOAD(venc_send, "CVI_VENC_SendFrame");
	LOAD(venc_get_stream, "CVI_VENC_GetStream");
	LOAD(venc_release_stream, "CVI_VENC_ReleaseStream");

	/* 帧缓冲布局：官方 helper（stride/plane/块大小/对齐都按芯片规则算） */
	VB_CAL_CONFIG_S vb_cfg;
	memset(&vb_cfg, 0, sizeof(vb_cfg));
	VENC_GetPicBufferConfig(width, height, HWJPEG_INPUT_FORMAT, DATA_BITWIDTH_8,
				COMPRESS_MODE_NONE, &vb_cfg);
	g.vb_size = vb_cfg.u32VBSize;
	g.stride = vb_cfg.u32MainStride;
	g.y_size = vb_cfg.u32MainYSize;
	g.c_stride = vb_cfg.u32CStride;
	g.c_size = vb_cfg.u32MainCSize;
	g.addr_align = vb_cfg.u16AddrAlign ? vb_cfg.u16AddrAlign : 1;
	g.plane_num = vb_cfg.plane_num;
	if (!g.vb_size || !g.stride) {
		set_err("VENC_GetPicBufferConfig 结果异常 (size=%u stride=%u)", g.vb_size, g.stride);
		return -1;
	}

	/* MMF 初始化：公共 VB 池（VENC 用户送帧模式从公共池取块） */
	CVI_S32 ret;

	g.sys_exit();
	g.vb_exit();
	VB_CONFIG_S vb;
	memset(&vb, 0, sizeof(vb));
	vb.u32MaxPoolCnt = 1;
	vb.astCommPool[0].u32BlkSize = g.vb_size;
	vb.astCommPool[0].u32BlkCnt = HWJPEG_VB_BLK_CNT;
	vb.astCommPool[0].enRemapMode = VB_REMAP_MODE_CACHED;
	snprintf(vb.astCommPool[0].acName, sizeof(vb.astCommPool[0].acName), "hwjpeg");
	ret = g.vb_set_config(&vb);
	if (ret != CVI_SUCCESS) {
		set_err("CVI_VB_SetConfig 失败: 0x%x (blkSize=%u)", ret, g.vb_size);
		return -1;
	}
	ret = g.vb_init();
	if (ret != CVI_SUCCESS) {
		set_err("CVI_VB_Init 失败: 0x%x", ret);
		return -1;
	}
	ret = g.sys_init();
	if (ret != CVI_SUCCESS) {
		set_err("CVI_SYS_Init 失败: 0x%x", ret);
		return -1;
	}

	/* JPEG 通道 */
	VENC_CHN_ATTR_S attr;
	memset(&attr, 0, sizeof(attr));
	attr.stVencAttr.enType = PT_JPEG;
	attr.stVencAttr.u32MaxPicWidth = width;
	attr.stVencAttr.u32MaxPicHeight = height;
	attr.stVencAttr.u32PicWidth = width;
	attr.stVencAttr.u32PicHeight = height;
	attr.stVencAttr.u32BufSize = width * height; /* 码流缓冲，640x480 JPEG 远小于此 */
	attr.stVencAttr.bByFrame = CVI_TRUE;
	attr.stVencAttr.stAttrJpege.bSupportDCF = CVI_FALSE;
	attr.stVencAttr.stAttrJpege.stMPFCfg.u8LargeThumbNailNum = 0;
	attr.stVencAttr.stAttrJpege.enReceiveMode = VENC_PIC_RECEIVE_SINGLE;
	attr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
	attr.stGopAttr.stNormalP.s32IPQpDelta = 0;
	ret = g.venc_create(HWJPEG_CHN, &attr);
	if (ret != CVI_SUCCESS) {
		set_err("CVI_VENC_CreateChn 失败: 0x%x（输入格式 %u 可能不被支持）", ret,
			(unsigned)HWJPEG_INPUT_FORMAT);
		return -1;
	}

	VENC_JPEG_PARAM_S jpeg;
	memset(&jpeg, 0, sizeof(jpeg));
	/* qfactor 1..99；50 是"用户量化表"特殊值，避开它 */
	uint32_t q = (quality >= 1 && quality <= 99) ? quality : 80;
	if (q == 50)
		q = 51;
	jpeg.u32Qfactor = q;
	jpeg.u32MCUPerECS = 0;
	ret = g.venc_set_jpeg(HWJPEG_CHN, &jpeg);
	if (ret != CVI_SUCCESS) {
		set_err("CVI_VENC_SetJpegParam 失败: 0x%x", ret);
		g.venc_destroy(HWJPEG_CHN);
		return -1;
	}

	VENC_RECV_PIC_PARAM_S recv;
	memset(&recv, 0, sizeof(recv));
	recv.s32RecvPicNum = -1; /* 不限帧数 */
	ret = g.venc_start(HWJPEG_CHN, &recv);
	if (ret != CVI_SUCCESS) {
		set_err("CVI_VENC_StartRecvFrame 失败: 0x%x", ret);
		g.venc_destroy(HWJPEG_CHN);
		return -1;
	}

	g.pts = 0;
	g.ready = 1;
	return 0;
}

/* 把 src（YUYV422，宽*2 字节/行）写进 VB 块：
 * - 输入格式为 YUYV 时直通拷贝；
 * - 为 NV12/NV21 时做 YUYV422 → semi-planar YUV420 转换（UV 每 2x2 取均值）。
 * 返回可用的虚拟地址。 */
static void *fill_block(VB_BLK blk, const uint8_t *src, uint32_t src_stride,
			VIDEO_FRAME_INFO_S *frame)
{
	CVI_U64 phy = g.vb_handle2phys(blk);
	void *vir = NULL;
	uint32_t uv_off = (g.y_size + g.addr_align - 1) / g.addr_align * g.addr_align;

	if (g.sys_mmap_cache)
		vir = g.sys_mmap_cache(phy, g.vb_size);
	if (!vir)
		vir = g.sys_mmap(phy, g.vb_size);
	if (!vir) {
		set_err("CVI_SYS_Mmap 失败 (phy=0x%llx len=%u)", (unsigned long long)phy, g.vb_size);
		return NULL;
	}

	if (HWJPEG_INPUT_FORMAT == PIXEL_FORMAT_YUYV) {
		if (g.stride == src_stride) {
			memcpy(vir, src, (size_t)g.stride * HWJPEG_HEIGHT);
		} else {
			uint32_t copy = g.stride < src_stride ? g.stride : src_stride;
			for (uint32_t y = 0; y < HWJPEG_HEIGHT; y++)
				memcpy((uint8_t *)vir + (size_t)y * g.stride,
				       src + (size_t)y * src_stride, copy);
		}
	} else {
		uint8_t *y_plane = (uint8_t *)vir;
		uint8_t *uv_plane = y_plane + uv_off;
		for (uint32_t y = 0; y < HWJPEG_HEIGHT; y += 2) {
			const uint8_t *r0 = src + (size_t)y * src_stride;
			const uint8_t *r1 = r0 + src_stride;
			uint8_t *d0 = y_plane + (size_t)y * g.stride;
			uint8_t *d1 = d0 + g.stride;
			uint8_t *uv = uv_plane + (size_t)(y / 2) * g.c_stride;
			for (uint32_t x = 0; x < HWJPEG_WIDTH / 2; x++) {
				d0[x * 2] = r0[x * 4];
				d0[x * 2 + 1] = r0[x * 4 + 2];
				d1[x * 2] = r1[x * 4];
				d1[x * 2 + 1] = r1[x * 4 + 2];
				uint8_t u = (uint8_t)(((uint32_t)r0[x * 4 + 1] + r1[x * 4 + 1]) / 2);
				uint8_t v = (uint8_t)(((uint32_t)r0[x * 4 + 3] + r1[x * 4 + 3]) / 2);
				uv[x * 2] = HWJPEG_SWAP_UV ? v : u;
				uv[x * 2 + 1] = HWJPEG_SWAP_UV ? u : v;
			}
		}
	}
	if (g.sys_flush)
		g.sys_flush(phy, vir, g.vb_size);

	memset(frame, 0, sizeof(*frame));
	frame->stVFrame.u32Width = HWJPEG_WIDTH;
	frame->stVFrame.u32Height = HWJPEG_HEIGHT;
	frame->stVFrame.enPixelFormat = HWJPEG_INPUT_FORMAT;
	frame->stVFrame.enVideoFormat = VIDEO_FORMAT_LINEAR;
	frame->stVFrame.enCompressMode = COMPRESS_MODE_NONE;
	frame->stVFrame.enDynamicRange = DYNAMIC_RANGE_SDR8;
	frame->stVFrame.enColorGamut = COLOR_GAMUT_BT709;
	frame->stVFrame.u32Stride[0] = g.stride;
	frame->stVFrame.u64PhyAddr[0] = phy;
	frame->stVFrame.pu8VirAddr[0] = (CVI_U8 *)vir;
	frame->stVFrame.u32Length[0] = g.y_size;
	if (g.plane_num > 1) {
		frame->stVFrame.u32Stride[1] = g.c_stride;
		frame->stVFrame.u64PhyAddr[1] = phy + uv_off;
		frame->stVFrame.pu8VirAddr[1] = (CVI_U8 *)vir + uv_off;
		frame->stVFrame.u32Length[1] = g.c_size;
	}
	frame->stVFrame.u64PTS = ++g.pts;
	frame->stVFrame.u32TimeRef = (CVI_U32)g.pts;
	frame->u32PoolId = g.vb_handle2pool(blk);
	return vir;
}

int hwjpeg_encode(const uint8_t *yuyv, uint32_t src_len, uint8_t *dst, uint32_t dst_cap,
		  uint32_t *out_len)
{
	if (!g.ready) {
		if (!g.err[0])
			set_err("未初始化");
		return -1;
	}
	if (!yuyv || src_len < (uint32_t)HWJPEG_WIDTH * 2 * HWJPEG_HEIGHT) {
		set_err("输入长度不足: %u < %u", src_len, HWJPEG_WIDTH * 2 * HWJPEG_HEIGHT);
		return -1;
	}
	if (out_len)
		*out_len = 0;

	VB_BLK blk = g.vb_get_block(VB_INVALID_POOLID, g.vb_size);
	if (blk == VB_INVALID_HANDLE) {
		set_err("CVI_VB_GetBlock 失败（VB 池耗尽？blkSize=%u）", g.vb_size);
		return -1;
	}

	VIDEO_FRAME_INFO_S frame;
	void *vir = fill_block(blk, yuyv, HWJPEG_WIDTH * 2, &frame);
	if (!vir) {
		g.vb_release_block(blk);
		return -1;
	}

	CVI_S32 ret = g.venc_send(HWJPEG_CHN, &frame, HWJPEG_TIMEOUT_MS);
	if (ret != CVI_SUCCESS) {
		set_err("CVI_VENC_SendFrame 失败: 0x%x（格式 %u 可能不被支持）", ret,
			(unsigned)HWJPEG_INPUT_FORMAT);
		g.sys_munmap(vir, g.vb_size);
		g.vb_release_block(blk);
		return -1;
	}

	VENC_STREAM_S stream;
	memset(&stream, 0, sizeof(stream));
	stream.pstPack = g.packs;
	stream.u32PackCount = HWJPEG_MAX_PACK;
	ret = g.venc_get_stream(HWJPEG_CHN, &stream, HWJPEG_TIMEOUT_MS);
	if (ret != CVI_SUCCESS) {
		set_err("CVI_VENC_GetStream 失败: 0x%x", ret);
		g.sys_munmap(vir, g.vb_size);
		g.vb_release_block(blk);
		return -1;
	}

	uint32_t total = 0;
	for (CVI_U32 i = 0; i < stream.u32PackCount; i++) {
		const VENC_PACK_S *p = &stream.pstPack[i];
		if (total + p->u32Len > dst_cap) {
			set_err("输出缓冲不足: 需要 %u > %u", total + p->u32Len, dst_cap);
			g.venc_release_stream(HWJPEG_CHN, &stream);
			g.sys_munmap(vir, g.vb_size);
			g.vb_release_block(blk);
			return -1;
		}
		memcpy(dst + total, p->pu8Addr, p->u32Len);
		total += p->u32Len;
	}

	g.venc_release_stream(HWJPEG_CHN, &stream);
	g.sys_munmap(vir, g.vb_size);
	g.vb_release_block(blk);

	if (out_len)
		*out_len = total;
	set_err("ok");
	return total > 0 ? 0 : -1;
}

void hwjpeg_exit(void)
{
	if (!g.ready)
		return;
	g.venc_stop(HWJPEG_CHN);
	g.venc_destroy(HWJPEG_CHN);
	g.sys_exit();
	g.vb_exit();
	g.ready = 0;
	if (g.h_venc)
		dlclose(g.h_venc);
	if (g.h_sys)
		dlclose(g.h_sys);
	g.h_venc = g.h_sys = NULL;
}

/* ------------------------------------------------------------------ */
/* 独立测试模式：读 640x480 YUYV422 裸图 → 编码 N 帧 → 写最后一帧 JPEG */
/* ------------------------------------------------------------------ */
#ifdef HWJPEG_MAIN
#include <time.h>

static double now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

int main(int argc, char **argv)
{
	const char *in = argc > 1 ? argv[1] : "/tmp/test.yuyv";
	const char *out = argc > 2 ? argv[2] : "/tmp/hwjpeg_out.jpg";
	int count = argc > 3 ? atoi(argv[3]) : 20;
	uint32_t quality = argc > 4 ? (uint32_t)atoi(argv[4]) : 80;
	size_t raw_len = HWJPEG_WIDTH * 2 * HWJPEG_HEIGHT;

	uint8_t *raw = malloc(raw_len);
	uint8_t *jpeg = malloc(1024 * 1024);
	if (!raw || !jpeg) {
		fprintf(stderr, "内存不足\n");
		return 2;
	}
	FILE *f = fopen(in, "rb");
	if (!f) {
		fprintf(stderr, "打不开输入 %s\n", in);
		return 2;
	}
	size_t got = fread(raw, 1, raw_len, f);
	fclose(f);
	if (got < raw_len) {
		fprintf(stderr, "输入不足: %zu < %zu\n", got, raw_len);
		return 2;
	}

	if (hwjpeg_init(HWJPEG_WIDTH, HWJPEG_HEIGHT, quality) != 0) {
		fprintf(stderr, "hwjpeg_init 失败: %s\n", hwjpeg_error());
		return 1;
	}
	printf("初始化 OK：输入格式=%u stride=%u VB 块=%u 字节，质量=%u\n",
	       hwjpeg_input_format(), g.stride, g.vb_size, quality);

	double t_all = now_ms();
	double t_first = 0;
	uint32_t out_len = 0;
	for (int i = 0; i < count; i++) {
		double t0 = now_ms();
		uint32_t n = 0;
		if (hwjpeg_encode(raw, (uint32_t)raw_len, jpeg, 1024 * 1024, &n) != 0) {
			fprintf(stderr, "第 %d 帧编码失败: %s\n", i, hwjpeg_error());
			hwjpeg_exit();
			return 1;
		}
		double dt = now_ms() - t0;
		if (i == 0)
			t_first = dt;
		out_len = n;
	}
	double total = now_ms() - t_all;
	printf("编码 %d 帧：首帧 %.1fms，平均 %.1fms/帧（%.1f fps），末帧 %u 字节\n",
	       count, t_first, total / count, count * 1000.0 / total, out_len);

	FILE *o = fopen(out, "wb");
	if (o) {
		fwrite(jpeg, 1, out_len, o);
		fclose(o);
		printf("已写出 %s\n", out);
	}
	hwjpeg_exit();
	free(raw);
	free(jpeg);
	return 0;
}
#endif /* HWJPEG_MAIN */
