//! `vpss_probe` —— Phase 0 探针：验证 SG2002 上「YUYV 进、RGB 平面 + NV12 出」的 VPSS 工作流。
//!
//! 目的（按顺序，每步打印返回码与耗时，失败即退出码 2）：
//!
//! 1. 建 `Sys`（VB 公共池）+ VPSS 组：输入 `YUYV`，chn0 `RGB_888_PLANAR`、chn1 `NV12`；
//! 2. 把一帧 640x480 YUYV（内置彩色图案或 `--input` 文件）写进 VB → `SendFrame` →
//!    两路 `GetChnFrame`，打印**帧描述符**（stride / 平面长度 / 物理地址是否连续）；
//! 3. 与 `sg2002_upper::preprocess::yuyv422_to_rgb` 的 CPU 参考对比（判断通道顺序与量程）；
//! 4. `--venc`：chn1 的 NV12 交给 `PT_JPEG` 硬编，出图写到 `<out>/vpss_chn1.jpg`；
//! 5. `--model <cvimodel>`：同一帧比较「memcpy 喂模型」与
//!    `CVI_NN_SetTensorPhysicalAddr(chn0.phy_addr(0))` 零拷贝的输出差异。
//!
//! 上板跑（务必加 `timeout`，VPSS 驱动没在这个 image 上跑过）：
//!
//! ```sh
//! timeout 20 ./vpss_probe --out-dir /root --venc
//! timeout 20 ./vpss_probe --step init          # 只建组/启组，验证驱动可用
//! timeout 30 ./vpss_probe --input /root/c1.yuv --venc --model /root/yolov8n_tennis_v3.cvimodel
//! ```

use std::io::Write;
use std::time::Instant;

use clap::Parser;
use cvimpi_rs::encoder::EncoderConfig;
use cvimpi_rs::ffi;
use cvimpi_rs::sys::{Sys, VbPoolConfig};
use cvimpi_rs::vpss::{VpssChnConfig, VpssConfig};
use sg2002_upper::preprocess::{FRAME_H, FRAME_W, YUYV_LEN, yuyv422_to_rgb};

/// YUYV 输入源。
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Pattern {
    /// 左上白 / 右上灰 / 左下红 / 右下蓝（判断通道顺序与量程）。
    Quad,
}

#[derive(Debug, Parser)]
#[command(name = "vpss_probe", version, about = "VPSS（YUYV→RGB/NV12）上板探针")]
struct Cli {
    /// 输出目录（帧 dump / JPEG 落这里）
    #[arg(long, default_value = "/root")]
    out_dir: String,

    /// YUYV 输入文件（640x480x2 = 614400 字节）；省略则用内置图案
    #[arg(long)]
    input: Option<String>,

    /// 内置图案（`--input` 优先）
    #[arg(long, value_enum, default_value_t = Pattern::Quad)]
    pattern: Pattern,

    /// 跑到哪一步就停：`init`（只建组）/ `frame`（建组+收发帧）/ `all`
    #[arg(long, default_value = "frame")]
    step: String,

    /// chn0 输出格式：`rgb-planar` / `bgr-planar` / `rgb-packed`
    #[arg(long, default_value = "rgb-planar")]
    chn0_format: String,

    /// 两个通道的用户态队列深度
    #[arg(long, default_value_t = 1)]
    depth: u32,

    /// `GetChnFrame` 超时（ms）
    #[arg(long, default_value_t = 1000)]
    timeout: i32,

    /// `SendFrame` 超时（ms；-1 = 阻塞）
    #[arg(long, default_value_t = 1000)]
    send_timeout: i32,

    /// chn0 输出 stride 对齐（0 = 不设置，用驱动默认）
    #[arg(long, default_value_t = 0)]
    align: u32,

    /// CSC 模式：`default` = 驱动默认（full range）/ `expand601` = BT.601
    /// limited→full（与 CPU 参考一致，模型输入分布不变）
    #[arg(long, default_value = "default")]
    csc: String,

    /// JPEG 质量（1..=99）
    #[arg(long, default_value_t = 70)]
    quality: u8,

    /// 跑 `PT_JPEG` 硬编（chn1 的 NV12 → JPEG）
    #[arg(long)]
    venc: bool,

    /// 跑多少帧（第 1 帧做完整检查，其余只跑流水线测吞吐）
    #[arg(long, default_value_t = 1)]
    frames: u32,

    /// chn1（NV12 预览）的降采样倍数：1 = 640x480，2 = 320x240（硬件缩放）
    #[arg(long, default_value_t = 1)]
    chn1_scale: u32,

    /// 模型路径：额外做 TPU 零拷贝对比
    #[arg(long)]
    model: Option<String>,

    /// VPSS 组号（默认自动：驱动实测一个组号一个 boot 只能建一次，见 `VPSS_GRP_AUTO`）
    #[arg(long)]
    grp: Option<i32>,

    /// `--step init` 成功后直接 `exit(0)`，**不**销毁组（也不走 SYS_Exit）
    #[arg(long)]
    leak: bool,
}

fn main() {
    let cli = Cli::parse();
    run(&cli);
}

fn run(cli: &Cli) {
    say("== vpss_probe：YUYV → VPSS → RGB_888_PLANAR + NV12 ==");
    let mut mode_ex = ffi::VPSS_MODE_S::default();
    let mode_rc = unsafe { ffi::CVI_SYS_GetVPSSModeEx(&mut mode_ex) };
    let mut vivpss = ffi::VI_VPSS_MODE_S::default();
    let vivpss_rc = unsafe { ffi::CVI_SYS_GetVIVPSSMode(&mut vivpss) };
    say(&format!(
        "  当前模式：GetVPSSModeEx rc={mode_rc} enMode={} input={:?} / GetVIVPSSMode rc={vivpss_rc} {:?}",
        mode_ex.enMode, mode_ex.aenInput, vivpss.aenMode
    ));

    /* 1) 输入帧 */
    let yuyv = load_yuyv(cli);
    say(&format!("  输入：{} 字节 YUYV", yuyv.len()));

    /* 2) VB 池尺寸（与 cvimpi-rs 的布局计算一致） */
    let rgb_fmt = parse_chn0_format(&cli.chn0_format);
    let yuyv_layout = expect(
        "venc_input_layout(YUYV)",
        cvimpi_rs::venc_input_layout(FRAME_W as u32, FRAME_H as u32, ffi::PIXEL_FORMAT_YUYV),
    );
    let rgb_layout = expect(
        "venc_input_layout(chn0)",
        cvimpi_rs::venc_input_layout(FRAME_W as u32, FRAME_H as u32, rgb_fmt),
    );
    let nv12_layout = expect(
        "venc_input_layout(NV12)",
        cvimpi_rs::venc_input_layout(FRAME_W as u32, FRAME_H as u32, ffi::PIXEL_FORMAT_NV12),
    );
    let blk_size = yuyv_layout
        .vb_size
        .max(rgb_layout.vb_size)
        .max(nv12_layout.vb_size);
    say(&format!(
        "  VB 池：blk={blk_size}（YUYV {} / chn0 {} / NV12 {}）× 6",
        yuyv_layout.vb_size, rgb_layout.vb_size, nv12_layout.vb_size
    ));

    /* 3) 会话 + 组 */
    let sys = match Sys::init(&[VbPoolConfig::new(blk_size, 6).with_name("vpss_probe")]) {
        Ok(sys) => {
            say("  [ok] CVI_SYS_Init + VB 池");
            sys
        }
        Err(e) => fail("Sys::init", &e.to_string()),
    };
    match sys.pools_match_request() {
        Some(true) => say("  内核池与请求一致"),
        Some(false) => say("  ⚠ 内核里是残留池（上次进程崩溃留下），块可能不够用"),
        None => say("  ⚠ 读不回内核池配置（继续）"),
    }

    let cfg = VpssConfig {
        grp: cli.grp.unwrap_or(cvimpi_rs::vpss::VPSS_GRP_AUTO),
        max_w: FRAME_W as u32,
        max_h: FRAME_H as u32,
        in_format: ffi::PIXEL_FORMAT_YUYV,
        chns: vec![
            VpssChnConfig::new(0, FRAME_W as u32, FRAME_H as u32, rgb_fmt).with_depth(cli.depth),
            VpssChnConfig::new(
                1,
                FRAME_W as u32 / cli.chn1_scale.max(1),
                FRAME_H as u32 / cli.chn1_scale.max(1),
                ffi::PIXEL_FORMAT_NV12,
            )
            .with_depth(cli.depth),
        ],
    };
    if cli.step == "twice" {
        // 同一进程里 create → drop → create：验证"一次 boot 只能建一次"是不是
        // 跨进程/生命周期造成的
        let first = sys.create_vpss(&cfg);
        say(&format!(
            "  同一进程第一次建组：{}",
            match &first {
                Ok(_) => "OK".to_string(),
                Err(e) => format!("FAIL {e}"),
            }
        ));
        drop(first);
        let second = sys.create_vpss(&cfg);
        say(&format!(
            "  同一进程第二次建组：{}",
            match &second {
                Ok(_) => "OK".to_string(),
                Err(e) => format!("FAIL {e}"),
            }
        ));
        return;
    }

    let t0 = Instant::now();
    let vpss = must(
        "CVI_VPSS_CreateGrp/SetChnAttr/EnableChn/StartGrp",
        sys.create_vpss(&cfg),
    );
    say(&format!(
        "  建组耗时 {:.1}ms",
        t0.elapsed().as_secs_f64() * 1000.0
    ));

    if cli.align != 0 {
        must(
            "CVI_VPSS_SetChnAlign(chn0)",
            vpss.set_chn_align(0, cli.align),
        );
    }
    match cli.csc.as_str() {
        "default" => {}
        "expand601" => {
            must(
                "CVI_VPSS_SetGrpCsc(601 limited→full)",
                vpss.set_yuv601_limited_to_full(),
            );
        }
        other => fail("--csc", &format!("未知取值 {other}（default / expand601）")),
    }

    if cli.step == "init" {
        say("== step=init：驱动可用（组已建好并启动）==");
        if cli.leak {
            say("  --leak：直接 exit(0)，不销毁组、不走 SYS_Exit/VB_Exit");
            std::process::exit(0);
        }
        drop(vpss);
        say("  [ok] 组已销毁");
        return;
    }

    /* 4) 写输入帧并送进组 */
    let mut frame = must(
        "alloc_frame_cached(YUYV)",
        sys.alloc_frame_cached(FRAME_W as u32, FRAME_H as u32, ffi::PIXEL_FORMAT_YUYV),
    );
    say(&format!(
        "  输入帧：stride={} len={} cached={}",
        frame.stride(0),
        frame.plane_len(0),
        frame.is_cached()
    ));
    must("write_tight(YUYV)", frame.write_tight(&yuyv));
    let t0 = Instant::now();
    must("Frame::flush", frame.flush());
    say(&format!("  写 VB + flush：{:.2}ms", ms_since(t0)));

    let t0 = Instant::now();
    must(
        "CVI_VPSS_SendFrame",
        vpss.send_frame(frame.info(), cli.send_timeout),
    );
    say(&format!("  SendFrame：{:.2}ms", ms_since(t0)));

    /* 5) 取两路输出 */
    let t0 = Instant::now();
    let mut rgb = must("GetChnFrame(chn0)", vpss.get_chn_frame(0, cli.timeout));
    say(&format!("  GetChnFrame(chn0)：{:.2}ms", ms_since(t0)));
    dump_frame("chn0", &rgb);
    let rgb_tight = must("copy_tight(chn0)", rgb.copy_tight());

    let t0 = Instant::now();
    let mut nv12 = must("GetChnFrame(chn1)", vpss.get_chn_frame(1, cli.timeout));
    say(&format!("  GetChnFrame(chn1)：{:.2}ms", ms_since(t0)));
    dump_frame("chn1", &nv12);
    let (nw, nh) = (nv12.width(), nv12.height());
    let nv12_tight = must("copy_tight(chn1)", nv12.copy_tight());
    nv12_stats(&nv12_tight, nw, nh);

    /* 6) 与 CPU 参考对比 + 落盘 */
    let mut reference = vec![0u8; FRAME_W * FRAME_H * 3];
    yuyv422_to_rgb(&yuyv, &mut reference);
    compare_rgb(&rgb_tight, &reference);

    let out = cli.out_dir.trim_end_matches('/');
    write_file(&format!("{out}/vpss_chn0_rgb.planar"), &rgb_tight);
    write_file(&format!("{out}/vpss_chn1_nv12.yuv"), &nv12_tight);
    write_file(&format!("{out}/ref_cpu_rgb.planar"), &reference);

    /* 7) NV12 → VENC(PT_JPEG) */
    let enc = if cli.venc {
        Some(must(
            "CVI_VENC_CreateChn(PT_JPEG, NV12)",
            sys.create_encoder(
                0,
                &EncoderConfig::new(FRAME_W as u32, FRAME_H as u32, ffi::PIXEL_FORMAT_NV12)
                    .with_quality(u32::from(cli.quality)),
            ),
        ))
    } else {
        None
    };
    if let Some(enc) = &enc {
        let t0 = Instant::now();
        let jpeg = must(
            "VENC SendFrame+GetStream(VPSS NV12)",
            enc.encode_info(nv12.info(), cli.timeout),
        );
        say(&format!(
            "  JPEG：{} 字节，{:.2}ms，SOI={} EOI={}",
            jpeg.len(),
            ms_since(t0),
            jpeg.starts_with(&[0xFF, 0xD8]),
            jpeg.ends_with(&[0xFF, 0xD9])
        ));
        write_file(&format!("{out}/vpss_chn1.jpg"), &jpeg);
    }

    /* 8) TPU：零拷贝 vs memcpy（同一帧），顺带比较 VPSS/CPU 两种 RGB */
    if let Some(model_path) = &cli.model {
        tp_vs_physical(model_path, &mut rgb, &rgb_tight, &reference);
    }

    /* 9) 多帧吞吐（可选） */
    if cli.frames > 1 {
        run_frames(cli, &sys, &vpss, &mut frame, &yuyv, enc.as_ref());
    }
    say("== 全部完成 ==");
}

/// 多帧流水线：CPU 搬运 → SendFrame → 取两路 →（可选）JPEG，测每帧耗时与吞吐。
fn run_frames(
    cli: &Cli,
    _sys: &Sys,
    vpss: &cvimpi_rs::vpss::Vpss<'_>,
    frame: &mut cvimpi_rs::encoder::Frame<'_>,
    yuyv: &[u8],
    enc: Option<&cvimpi_rs::encoder::Encoder<'_>>,
) {
    let n = cli.frames;
    say(&format!("== 多帧吞吐：{n} 帧 =="));
    // `--model` 给了就每帧跑一次零拷贝推理（应用的真实形态：模型 + 预览）
    let model = cli.model.as_ref().map(|path| {
        let m = match cviruntime_rs::Model::from_file(path) {
            Ok(m) => m,
            Err(e) => fail("cviruntime Model::from_file", &e.to_string()),
        };
        say("  [ok] 模型已加载（零拷贝在环）");
        m
    });
    let (mut t_write, mut t_send, mut t_get0, mut t_get1, mut t_venc) = (0.0, 0.0, 0.0, 0.0, 0.0);
    let mut t_model = 0.0;
    let t_all = Instant::now();
    for i in 0..n {
        // 每帧都重写输入（模拟相机 memcpy），并做一点变化避免内容完全相同
        let t0 = Instant::now();
        let mut buf = yuyv.to_vec();
        if i > 0 {
            let shift = (i as usize * 977) % FRAME_H;
            buf.rotate_left(shift * FRAME_W * 2);
        }
        must("write_tight", frame.write_tight(&buf));
        must("flush", frame.flush());
        t_write += ms_since(t0);

        let t0 = Instant::now();
        must("SendFrame", vpss.send_frame(frame.info(), cli.send_timeout));
        t_send += ms_since(t0);

        let t0 = Instant::now();
        let f0 = must("GetChnFrame(chn0)", vpss.get_chn_frame(0, cli.timeout));
        t_get0 += ms_since(t0);
        if let Some(model) = &model {
            let t0 = Instant::now();
            match model.forward_physical(f0.phy_addr(0)) {
                Ok(_) => t_model += ms_since(t0),
                Err(e) => fail("Model::forward_physical", &e.to_string()),
            }
        }
        must("ReleaseChnFrame(chn0)", f0.release());

        let t0 = Instant::now();
        let f1 = must("GetChnFrame(chn1)", vpss.get_chn_frame(1, cli.timeout));
        t_get1 += ms_since(t0);
        if let Some(enc) = enc {
            let t0 = Instant::now();
            must("VENC encode", enc.encode_info(f1.info(), cli.timeout));
            t_venc += ms_since(t0);
        }
        must("ReleaseChnFrame(chn1)", f1.release());
    }
    let total = ms_since(t_all);
    let per = |v: f64| v / f64::from(n);
    say(&format!(
        "  每帧：写VB {:.2} + SendFrame {:.2} + GetChn0 {:.2} + TPU {:.2} + GetChn1 {:.2} + VENC {:.2} ms",
        per(t_write),
        per(t_send),
        per(t_get0),
        per(t_model),
        per(t_get1),
        per(t_venc)
    ));
    say(&format!(
        "  总计 {total:.1}ms / {n} 帧 = {:.1} fps（含取帧后的业务处理）",
        f64::from(n) / (total / 1000.0)
    ));
}

/// 用同一帧比较「memcpy 进张量」与「`SetTensorPhysicalAddr` 零拷贝」的输出，
/// 顺带对比 VPSS 的 RGB（full-range CSC）与 CPU 参考 RGB 的检测差异。
///
/// 顺序很重要：`CVI_NN_SetTensorPhysicalAddr` 会释放张量原本的内存，
/// 所以先跑完所有 memcpy 路径，最后再切到物理地址路径。
fn tp_vs_physical(
    model_path: &str,
    rgb: &mut cvimpi_rs::vpss::VpssFrame<'_>,
    rgb_tight: &[u8],
    cpu_reference: &[u8],
) {
    let model = match cviruntime_rs::Model::from_file(model_path) {
        Ok(m) => {
            say("  [ok] 模型加载");
            m
        }
        Err(e) => fail("cviruntime Model::from_file", &e.to_string()),
    };
    let (in_shape, out_shape) = (
        format!("{:?}", model.inputs[0].shape),
        format!("{:?}", model.outputs[0].shape),
    );
    say(&format!(
        "  in={in_shape} ({} 字节)  out={out_shape} ({} 字节)",
        model.inputs[0].bytes, model.outputs[0].bytes
    ));
    if model.inputs[0].bytes != rgb_tight.len() {
        fail(
            "模型输入尺寸",
            &format!(
                "张量 {} 字节 != VPSS RGB {} 字节（0 拷贝要求完全一致）",
                model.inputs[0].bytes,
                rgb_tight.len()
            ),
        );
    }

    let t0 = Instant::now();
    let out_memcpy = match model.forward(rgb_tight) {
        Ok(o) => o.to_vec(),
        Err(e) => fail("Model::forward(memcpy)", &e.to_string()),
    };
    say(&format!(
        "  memcpy 路径（VPSS RGB）：{:.2}ms，置信度峰值 {:.4}",
        ms_since(t0),
        max_conf(&out_memcpy)
    ));

    let t0 = Instant::now();
    let out_cpu = match model.forward(cpu_reference) {
        Ok(o) => o.to_vec(),
        Err(e) => fail("Model::forward(CPU RGB)", &e.to_string()),
    };
    let cpu_diff = out_memcpy
        .iter()
        .zip(&out_cpu)
        .map(|(a, b)| (*a as i32 - *b as i32).abs())
        .max()
        .unwrap_or(0);
    say(&format!(
        "  memcpy 路径（CPU 参考 RGB）：{:.2}ms，置信度峰值 {:.4}（与 VPSS RGB 输出最大差 {cpu_diff}）",
        ms_since(t0),
        max_conf(&out_cpu),
    ));

    let paddr = rgb.phy_addr(0);
    let t0 = Instant::now();
    let out_paddr = match model.forward_physical(paddr) {
        Ok(o) => o.to_vec(),
        Err(e) => fail("Model::forward_physical", &e.to_string()),
    };
    say(&format!(
        "  零拷贝路径：{:.2}ms（paddr={paddr:#x}）",
        ms_since(t0)
    ));

    // 输出对比：逐字节最大差 + 置信度通道峰值
    let max_diff = out_memcpy
        .iter()
        .zip(&out_paddr)
        .map(|(a, b)| (*a as i32 - *b as i32).abs())
        .max()
        .unwrap_or(0);
    say(&format!(
        "  输出最大字节差 = {max_diff}（0 = 两条路径完全一致）"
    ));
    say(&format!(
        "  置信度峰值：memcpy {:.4} vs 零拷贝 {:.4}",
        max_conf(&out_memcpy),
        max_conf(&out_paddr)
    ));
}

fn max_conf(out: &[u8]) -> f32 {
    // YOLOv8 输出 [1,5,6300,1]：通道 4 是置信度。
    let count = out.len() / 4;
    let anchors = count / 5;
    if anchors == 0 {
        return f32::NAN;
    }
    let mut best = 0.0f32;
    for i in (4 * anchors)..(5 * anchors) {
        let v = f32::from_ne_bytes([out[i * 4], out[i * 4 + 1], out[i * 4 + 2], out[i * 4 + 3]]);
        if v > best {
            best = v;
        }
    }
    best
}

/* ------------------------------------------------------------------ */
/* 工具                                                                */
/* ------------------------------------------------------------------ */

/// 640x480 YUYV：`--input` 文件优先，否则用内置图案。
fn load_yuyv(cli: &Cli) -> Vec<u8> {
    if let Some(path) = &cli.input {
        let data = std::fs::read(path).unwrap_or_else(|e| {
            eprintln!("读不到输入文件 {path}: {e}");
            std::process::exit(2);
        });
        if data.len() < YUYV_LEN {
            eprintln!(
                "{path} 只有 {} 字节，少于一帧 YUYV 的 {YUYV_LEN}",
                data.len()
            );
            std::process::exit(2);
        }
        return data[..YUYV_LEN].to_vec();
    }
    match cli.pattern {
        Pattern::Quad => quad_pattern(),
    }
}

/// 左上白 / 右上灰 / 左下红 / 右下蓝（BT.601 limited 的 YUV 值）。
fn quad_pattern() -> Vec<u8> {
    const WHITE: [u8; 3] = [235, 128, 128];
    const GRAY: [u8; 3] = [128, 128, 128];
    const RED: [u8; 3] = [82, 90, 240];
    const BLUE: [u8; 3] = [41, 240, 110];
    let mut buf = vec![0u8; YUYV_LEN];
    for y in 0..FRAME_H {
        for pair in 0..FRAME_W / 2 {
            let x = pair * 2;
            let px = if y < FRAME_H / 2 {
                if x < FRAME_W / 2 { WHITE } else { GRAY }
            } else if x < FRAME_W / 2 {
                RED
            } else {
                BLUE
            };
            // YUYV：一组 4 字节，两个像素共享 U/V（组内必在同一象限，边界 320 是偶数）
            let off = y * FRAME_W * 2 + pair * 4;
            buf[off] = px[0];
            buf[off + 1] = px[1];
            buf[off + 2] = px[0];
            buf[off + 3] = px[2];
        }
    }
    buf
}

fn parse_chn0_format(name: &str) -> ffi::PIXEL_FORMAT_E {
    match name {
        "rgb-planar" => ffi::PIXEL_FORMAT_RGB_888_PLANAR,
        "bgr-planar" => ffi::PIXEL_FORMAT_BGR_888_PLANAR,
        "rgb-packed" => ffi::PIXEL_FORMAT_RGB_888,
        other => {
            eprintln!("不支持的 chn0 格式 {other}（rgb-planar / bgr-planar / rgb-packed）");
            std::process::exit(2);
        }
    }
}

fn dump_frame(tag: &str, f: &cvimpi_rs::vpss::VpssFrame<'_>) {
    say(&format!(
        "  {tag}: {}x{} fmt={} planes={}",
        f.width(),
        f.height(),
        f.pixel_format(),
        f.plane_count()
    ));
    for p in 0..f.plane_count() {
        say(&format!(
            "    plane{p}: phy={:#x} stride={} len={}",
            f.phy_addr(p),
            f.stride(p),
            f.plane_len(p)
        ));
    }
    // 3 平面时检查物理地址是否连续（TPU 零拷贝的前提）
    if f.plane_count() == 3 {
        let contiguous = f.phy_addr(1) == f.phy_addr(0) + u64::from(f.plane_len(0))
            && f.phy_addr(2) == f.phy_addr(1) + u64::from(f.plane_len(1));
        say(&format!(
            "    平面连续 = {contiguous}（TPU 零拷贝需要 true 且 stride == 宽度）"
        ));
    }
}

/// 与 CPU 参考逐通道对比：均值（判断量程）、mean|diff|（判断精度）、左右半均值（判断 R/B 顺序）。
fn compare_rgb(got: &[u8], want: &[u8]) {
    let plane = FRAME_W * FRAME_H;
    if got.len() < plane * 3 || want.len() < plane * 3 {
        say("  ⚠ RGB 缓冲长度不足，跳过对比");
        return;
    }
    say("  与 CPU 参考对比（R/G/B）：");
    for (i, name) in ["R", "G", "B"].iter().enumerate() {
        let g = &got[i * plane..(i + 1) * plane];
        let w = &want[i * plane..(i + 1) * plane];
        let mean = |s: &[u8]| s.iter().map(|&x| f64::from(x)).sum::<f64>() / s.len() as f64;
        let diff = g
            .iter()
            .zip(w)
            .map(|(a, b)| f64::from((i32::from(*a) - i32::from(*b)).unsigned_abs()))
            .sum::<f64>()
            / g.len() as f64;
        let half = |s: &[u8], right: bool| {
            let mut sum = 0.0;
            let mut n = 0usize;
            for y in 0..FRAME_H {
                let xs = if right {
                    FRAME_W / 2..FRAME_W
                } else {
                    0..FRAME_W / 2
                };
                for x in xs {
                    sum += f64::from(s[y * FRAME_W + x]);
                    n += 1;
                }
            }
            sum / n as f64
        };
        say(&format!(
            "    {name}: VPSS 均值 {:.1} / 参考 {:.1} / mean|diff| {:.1}；VPSS 左 {:.1} 右 {:.1}，参考 左 {:.1} 右 {:.1}",
            mean(g),
            mean(w),
            diff,
            half(g, false),
            half(g, true),
            half(w, false),
            half(w, true),
        ));
    }
}

/// NV12 的 Y/U/V 均值：用来判断组 CSC 是否也作用在 YUV 输出通道上
/// （如果加了 `--csc expand601` 之后 NV12 的 Y 均值也变了，说明会波及预览通道）。
fn nv12_stats(nv12: &[u8], w: u32, h: u32) {
    let y_plane = (w * h) as usize;
    if nv12.len() < y_plane + y_plane / 2 {
        say("  ⚠ NV12 长度不足，跳过统计");
        return;
    }
    let mean = |s: &[u8]| s.iter().map(|&x| f64::from(x)).sum::<f64>() / s.len() as f64;
    let y = mean(&nv12[..y_plane]);
    let uv = &nv12[y_plane..y_plane + y_plane / 2];
    let u = uv.iter().step_by(2).copied().collect::<Vec<_>>();
    let v = uv.iter().skip(1).step_by(2).copied().collect::<Vec<_>>();
    say(&format!(
        "  NV12({w}x{h}) 均值：Y {:.1} / U {:.1} / V {:.1}",
        y,
        mean(&u),
        mean(&v)
    ));
}

fn write_file(path: &str, data: &[u8]) {
    match std::fs::write(path, data) {
        Ok(()) => say(&format!("  [ok] 写出 {path}（{} 字节）", data.len())),
        Err(e) => say(&format!("  ⚠ 写 {path} 失败：{e}")),
    }
}

fn ms_since(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1000.0
}

/// 打印一行并立刻刷出（上板跑，前面可能就卡死了）。
fn say(line: &str) {
    println!("{line}");
    let _ = std::io::stdout().flush();
}

fn expect<T>(what: &str, v: Option<T>) -> T {
    match v {
        Some(v) => v,
        None => fail(what, "布局计算返回 None（不支持的像素格式）"),
    }
}

/// `cvimpi_rs::Result` 失败即退出（带 errno 文本）。
fn must<T>(what: &str, r: cvimpi_rs::Result<T>) -> T {
    match r {
        Ok(v) => {
            say(&format!("  [ok] {what}"));
            v
        }
        Err(e) => {
            let extra = match cvimpi_rs::vpss::errno_str(&e) {
                Some(s) => format!("（{s}）"),
                None => String::new(),
            };
            eprintln!("  [FAIL] {what}: {e}{extra}");
            std::process::exit(2);
        }
    }
}

fn fail(what: &str, msg: &str) -> ! {
    eprintln!("  [FAIL] {what}: {msg}");
    std::process::exit(2);
}
