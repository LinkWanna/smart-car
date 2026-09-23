//! 硬件 MJPEG 解码测速探针（SG2002 VDEC / PT_MJPEG）。
//!
//! 用来回答「相机出 MJPG + 硬件解码」这条路线值不值：解码一张 640x480 MJPEG，
//! 输出 planar YUV（444/420 由驱动决定），报告每帧耗时与输出的像素格式/stride。
//!
//! 用法：
//!   riscv64-linux-musl-gcc -O2 -I<sdk>/include -I<sdk>/include/linux -o hwdec_probe csrc/hwdec_probe.c
//!   LD_LIBRARY_PATH=/mnt/system/usr/lib ./hwdec_probe frame.mjpg out.yuv 30
//!
//! 输出 `out.yuv` 是裸 YUV 平面（按打印出的格式用 ffmpeg 转 PNG 核对）。

#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "cvi_buffer.h"
#include "cvi_comm_vb.h"
#include "cvi_sys.h"
#include "cvi_vb.h"
#include "cvi_vdec.h"

#define WIDTH 640
#define HEIGHT 480
#define CHN 0
#define TIMEOUT_MS 2000

typedef CVI_S32 (*fn_void_int)(void);
typedef void *(*fn_mmap)(CVI_U64, CVI_U32);
typedef CVI_S32 (*fn_munmap)(void *, CVI_U32);
typedef CVI_S32 (*fn_vb_set)(const VB_CONFIG_S *);
typedef VB_BLK (*fn_vb_get)(VB_POOL, CVI_U32);
typedef CVI_S32 (*fn_vb_rel)(VB_BLK);
typedef CVI_S32 (*fn_venc_like_1)(VDEC_CHN, const VDEC_CHN_ATTR_S *);
typedef CVI_S32 (*fn_venc_like_2)(VDEC_CHN);
typedef CVI_S32 (*fn_send)(VDEC_CHN, const VDEC_STREAM_S *, CVI_S32);
typedef CVI_S32 (*fn_get)(VDEC_CHN, VIDEO_FRAME_INFO_S *, CVI_S32);
typedef CVI_S32 (*fn_rel)(VDEC_CHN, const VIDEO_FRAME_INFO_S *);
typedef CVI_S32 (*fn_get_param)(VDEC_CHN, VDEC_CHN_PARAM_S *);
typedef CVI_S32 (*fn_set_param)(VDEC_CHN, const VDEC_CHN_PARAM_S *);

static void *h_sys, *h_dec;

static double now_ms(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static void *sym(void *h, const char *name)
{
	void *p = dlsym(h, name);
	if (!p)
		fprintf(stderr, "dlsym %s 失败: %s\n", name, dlerror());
	return p;
}

/* 从文件里取出第一帧 JPEG（SOI...EOI）。 */
static uint8_t *read_first_jpeg(const char *path, uint32_t *out_len)
{
	FILE *f = fopen(path, "rb");
	if (!f) {
		fprintf(stderr, "打不开 %s\n", path);
		return NULL;
	}
	fseek(f, 0, SEEK_END);
	long n = ftell(f);
	fseek(f, 0, SEEK_SET);
	uint8_t *buf = malloc(n);
	if (fread(buf, 1, n, f) != (size_t)n) {
		free(buf);
		fclose(f);
		return NULL;
	}
	fclose(f);
	uint32_t start = 0, end = 0;
	for (long i = 0; i + 1 < n; i++) {
		if (buf[i] == 0xFF && buf[i + 1] == 0xD8 && start == 0) {
			start = i;
		} else if (buf[i] == 0xFF && buf[i + 1] == 0xD9) {
			end = i + 2;
			break;
		}
	}
	if (!end) {
		free(buf);
		return NULL;
	}
	*out_len = end - start;
	memmove(buf, buf + start, *out_len);
	return buf;
}

int main(int argc, char **argv)
{
	const char *in = argc > 1 ? argv[1] : "/tmp/frame.mjpg";
	const char *out = argc > 2 ? argv[2] : "/tmp/dec.yuv";
	int count = argc > 3 ? atoi(argv[3]) : 30;
	/* 可选调参：buf_cnt 与块大小倍率（默认 3 块、1.0x，遇到 NOMEM 时用来试探） */
	CVI_U32 buf_cnt = argc > 4 ? (CVI_U32)atoi(argv[4]) : 3;
	double blk_scale = argc > 5 ? atof(argv[5]) : 1.0;
	int fmt_arg = argc > 6 ? atoi(argv[6]) : 444; /* 444 或 12(NV12) */
	CVI_U32 stream_arg = argc > 7 ? (CVI_U32)strtoul(argv[7], NULL, 0) : 0;
	CVI_U32 pic_w = argc > 8 ? (CVI_U32)atoi(argv[8]) : WIDTH;
	CVI_U32 pic_h = argc > 9 ? (CVI_U32)atoi(argv[9]) : HEIGHT;

	uint32_t jpeg_len = 0;
	uint8_t *jpeg = read_first_jpeg(in, &jpeg_len);
	if (!jpeg) {
		fprintf(stderr, "输入里没找到完整 JPEG\n");
		return 2;
	}
	printf("输入 MJPEG 帧：%u 字节\n", jpeg_len);

	/* 库加载：libatomic（libsys 需要 1 字节 CAS）→ libsys → libvdec */
	dlopen("/usr/lib/libatomic.so.1", RTLD_NOW | RTLD_GLOBAL);
	dlopen("/lib/libatomic.so.1", RTLD_NOW | RTLD_GLOBAL);
	h_sys = dlopen("/mnt/system/usr/lib/libsys.so", RTLD_NOW | RTLD_GLOBAL);
	if (!h_sys) {
		fprintf(stderr, "dlopen libsys 失败: %s\n", dlerror());
		return 1;
	}
	h_dec = dlopen("/mnt/system/usr/lib/libvdec.so", RTLD_NOW | RTLD_GLOBAL);
	if (!h_dec) {
		fprintf(stderr, "dlopen libvdec 失败: %s\n", dlerror());
		return 1;
	}

	fn_void_int sys_init = (fn_void_int)sym(h_sys, "CVI_SYS_Init");
	fn_void_int sys_exit = (fn_void_int)sym(h_sys, "CVI_SYS_Exit");
	fn_mmap sys_mmap = (fn_mmap)sym(h_sys, "CVI_SYS_Mmap");
	fn_munmap sys_munmap = (fn_munmap)sym(h_sys, "CVI_SYS_Munmap");
	fn_vb_set vb_set = (fn_vb_set)sym(h_sys, "CVI_VB_SetConfig");
	fn_void_int vb_init = (fn_void_int)sym(h_sys, "CVI_VB_Init");
	fn_void_int vb_exit = (fn_void_int)sym(h_sys, "CVI_VB_Exit");
	fn_venc_like_1 dec_create = (fn_venc_like_1)sym(h_dec, "CVI_VDEC_CreateChn");
	fn_venc_like_2 dec_start = (fn_venc_like_2)sym(h_dec, "CVI_VDEC_StartRecvStream");
	fn_venc_like_2 dec_stop = (fn_venc_like_2)sym(h_dec, "CVI_VDEC_StopRecvStream");
	fn_venc_like_2 dec_destroy = (fn_venc_like_2)sym(h_dec, "CVI_VDEC_DestroyChn");
	fn_send dec_send = (fn_send)sym(h_dec, "CVI_VDEC_SendStream");
	fn_get dec_get = (fn_get)sym(h_dec, "CVI_VDEC_GetFrame");
	fn_rel dec_rel = (fn_rel)sym(h_dec, "CVI_VDEC_ReleaseFrame");
	fn_get_param dec_get_param = (fn_get_param)sym(h_dec, "CVI_VDEC_GetChnParam");
	fn_set_param dec_set_param = (fn_set_param)sym(h_dec, "CVI_VDEC_SetChnParam");
	if (!sys_init || !dec_create || !dec_send || !dec_get || !dec_rel)
		return 1;

	/* 输出按 planar 444（JPEG/MJPEG 解码的原生输出），缓冲大小用官方 helper 算 */
	PIXEL_FORMAT_E out_fmt =
		fmt_arg == 12 ? PIXEL_FORMAT_NV12 : PIXEL_FORMAT_YUV_PLANAR_444;
	CVI_U32 frame_buf_size = VDEC_GetPicBufferSize(PT_MJPEG, pic_w, pic_h, out_fmt,
						      DATA_BITWIDTH_8, COMPRESS_MODE_NONE);
	if (blk_scale != 1.0)
		frame_buf_size = (CVI_U32)(frame_buf_size * blk_scale);
	/* 块大小按 64KB 对齐（驱动内部要求） */
	frame_buf_size = (frame_buf_size + 0xFFFF) & ~0xFFFFu;
	CVI_U32 stream_buf_size = stream_arg ? stream_arg
					     : (((pic_w * pic_h) + 0x3FFF) & ~0x3FFFu);
	printf("输出格式=%d 帧缓冲=%u 字节(%u 块) 码流缓冲=%u 字节\n", out_fmt, frame_buf_size,
	       buf_cnt, stream_buf_size);

	sys_exit();
	vb_exit();
	VB_CONFIG_S vb;
	memset(&vb, 0, sizeof(vb));
	vb.u32MaxPoolCnt = 1;
	vb.astCommPool[0].u32BlkSize = frame_buf_size;
	vb.astCommPool[0].u32BlkCnt = buf_cnt;
	vb.astCommPool[0].enRemapMode = VB_REMAP_MODE_CACHED;
	if (vb_set(&vb) != CVI_SUCCESS || vb_init() != CVI_SUCCESS) {
		fprintf(stderr, "VB 初始化失败\n");
		return 1;
	}
	if (sys_init() != CVI_SUCCESS) {
		fprintf(stderr, "CVI_SYS_Init 失败\n");
		return 1;
	}

	VDEC_CHN_ATTR_S attr;
	memset(&attr, 0, sizeof(attr));
	attr.enType = PT_MJPEG;
	attr.enMode = VIDEO_MODE_FRAME;
	attr.u32PicWidth = pic_w;
	attr.u32PicHeight = pic_h;
	attr.u32StreamBufSize = stream_buf_size;
	attr.u32FrameBufSize = frame_buf_size;
	attr.u32FrameBufCnt = buf_cnt;
	CVI_S32 ret = dec_create(CHN, &attr);
	if (ret != CVI_SUCCESS) {
		fprintf(stderr, "CVI_VDEC_CreateChn 失败: 0x%x\n", ret);
		return 1;
	}
	/* 关键：输出像素格式通过 Get/SetChnParam 设置（不设会 NOMEM） */
	if (dec_get_param && dec_set_param) {
		VDEC_CHN_PARAM_S param;
		memset(&param, 0, sizeof(param));
		ret = dec_get_param(CHN, &param);
		if (ret != CVI_SUCCESS)
			fprintf(stderr, "GetChnParam 失败: 0x%x\n", ret);
		param.enPixelFormat = out_fmt;
		param.stVdecPictureParam.u32Alpha = 255;
		ret = dec_set_param(CHN, &param);
		if (ret != CVI_SUCCESS) {
			fprintf(stderr, "SetChnParam 失败: 0x%x\n", ret);
			return 1;
		}
	} else {
		fprintf(stderr, "缺少 Get/SetChnParam\n");
		return 1;
	}

	ret = dec_start(CHN);
	if (ret != CVI_SUCCESS) {
		fprintf(stderr, "CVI_VDEC_StartRecvStream 失败: 0x%x\n", ret);
		return 1;
	}

	double t_all = now_ms();
	double t_first = 0, t_send = 0, t_get = 0;
	uint32_t dumped = 0;
	for (int i = 0; i < count; i++) {
		VDEC_STREAM_S stream;
		memset(&stream, 0, sizeof(stream));
		stream.pu8Addr = jpeg;
		stream.u32Len = jpeg_len;
		stream.u64PTS = (CVI_U64)i;
		stream.bEndOfFrame = CVI_TRUE;
		stream.bEndOfStream = CVI_FALSE;
		stream.bDisplay = CVI_TRUE;

		double t0 = now_ms();
		ret = dec_send(CHN, &stream, TIMEOUT_MS);
		double t1 = now_ms();
		if (ret != CVI_SUCCESS) {
			fprintf(stderr, "SendStream 失败: 0x%x\n", ret);
			break;
		}
		VIDEO_FRAME_INFO_S frame;
		memset(&frame, 0, sizeof(frame));
		ret = dec_get(CHN, &frame, TIMEOUT_MS);
		double t2 = now_ms();
		if (ret != CVI_SUCCESS) {
			fprintf(stderr, "GetFrame 失败: 0x%x\n", ret);
			break;
		}
		if (i == 0) {
			t_first = t2 - t0;
			printf("解码输出：格式=%d %ux%u stride=[%u,%u,%u] length=[%u,%u,%u]\n",
			       frame.stVFrame.enPixelFormat, frame.stVFrame.u32Width,
			       frame.stVFrame.u32Height, frame.stVFrame.u32Stride[0],
			       frame.stVFrame.u32Stride[1], frame.stVFrame.u32Stride[2],
			       frame.stVFrame.u32Length[0], frame.stVFrame.u32Length[1],
			       frame.stVFrame.u32Length[2]);
		}
		/* 最后一帧把三个平面 dump 出来（按 stride 逐行，便于 ffmpeg 核对） */
		if (i == count - 1) {
			FILE *o = fopen(out, "wb");
			if (o) {
				for (int p = 0; p < 3; p++) {
					uint32_t stride = frame.stVFrame.u32Stride[p];
					uint32_t rows = frame.stVFrame.u32Height;
					uint32_t cols = p == 0 ? WIDTH : WIDTH; /* 444：三平面同尺寸 */
					if (!frame.stVFrame.u64PhyAddr[p] || !stride)
						continue;
					uint8_t *vir = (uint8_t *)sys_mmap(frame.stVFrame.u64PhyAddr[p],
									   stride * rows);
					if (!vir)
						continue;
					for (uint32_t y = 0; y < rows; y++)
						fwrite(vir + (size_t)y * stride, 1, cols, o);
					sys_munmap(vir, stride * rows);
				}
				fclose(o);
				dumped = 1;
			}
		}
		dec_rel(CHN, &frame);
		t_send += t1 - t0;
		t_get += t2 - t1;
	}
	double total = now_ms() - t_all;

	printf("解码 %d 帧：首帧 %.1fms，平均 %.2fms/帧（%.0f fps）"
	       " [Send %.2fms / GetFrame %.2fms]\n",
	       count, t_first, total / count, count * 1000.0 / total, t_send / count,
	       t_get / count);
	if (dumped)
		printf("已写出解码 YUV: %s\n", out);

	dec_stop(CHN);
	dec_destroy(CHN);
	sys_exit();
	vb_exit();
	free(jpeg);
	return 0;
}
