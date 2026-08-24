#!/usr/bin/env python3
"""Create a 6-slide group-meeting PPTX for the Adaptive FAST-LIO2 module.

This script writes a minimal OOXML PowerPoint file directly, avoiding external
dependencies such as python-pptx.
"""

from __future__ import annotations

import csv
import html
import shutil
import zipfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "experiments/subt_mrs_hawkins_long_corridor/results"
OUT = RESULTS / "adaptive_fast_lio2_group_meeting_6slides.pptx"
TMP = ROOT / "tmp_pptx_build"
WINDOW_EXAMPLE = RESULTS / "sliding_window_example.png"

SLIDE_W = 13_333_333
SLIDE_H = 7_500_000


def emu(inch: float) -> int:
    return int(inch * 914400)


def esc(s: str) -> str:
    return html.escape(s, quote=False)


def read_metrics() -> dict[tuple[str, str], dict[str, float]]:
    path = RESULTS / "evo_ape_ate_latest_comparison.csv"
    out: dict[tuple[str, str], dict[str, float]] = {}
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            out[(row["metric"], row["method"])] = {
                k: float(row[k])
                for k in ("rmse_m", "mean_m", "median_m", "max_m")
            }
    return out


def make_window_example(path: Path) -> None:
    """Draw a compact example for the sliding-window persistent test."""
    path.parent.mkdir(parents=True, exist_ok=True)
    img = Image.new("RGB", (1800, 290), "white")
    draw = ImageDraw.Draw(img)

    def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
        candidates = [
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
            "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
            "/usr/share/fonts/truetype/arphic/uming.ttc",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        ]
        if bold:
            candidates = [
                "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
                "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            ]
        for c in candidates:
            if Path(c).exists():
                return ImageFont.truetype(c, size)
        return ImageFont.load_default()

    font_box = font(28, True)
    font_sub = font(28)

    black = (31, 41, 55)
    gray = (107, 114, 128)
    light = (243, 244, 246)
    orange = (245, 158, 11)
    red = (220, 38, 38)

    # A realistic sequence: early normal frames, then a long corridor-like degenerate segment.
    seq = list("NNNDDDNDDDDDDDDDDDDD")
    x0, y0 = 65, 58
    box_w, gap = 72, 10
    for i, s in enumerate(seq):
        x = x0 + i * (box_w + gap)
        color = orange if s == "D" else light
        outline = red if s == "D" else (209, 213, 219)
        draw.rounded_rectangle((x, y0, x + box_w, y0 + 68), radius=11, fill=color, outline=outline, width=3)
        draw.text((x + 26, y0 + 17), s, fill=black, font=font_box)
        draw.text((x + 23, y0 + 82), str(i + 1), fill=gray, font=font_sub)

    img.save(path)


def make_map_comparison_from_three(output: Path, fastlio: Path, liosam: Path, ours: Path) -> None:
    """Combine three map screenshots into a single comparison image."""
    imgs = [Image.open(p).convert("RGB") for p in (fastlio, liosam, ours)]
    target_h = 520
    resized = []
    for im in imgs:
        scale = target_h / im.height
        resized.append(im.resize((int(im.width * scale), target_h)))

    labels = ["FAST-LIO", "LIO-SAM", "Ours"]
    gap = 22
    top = 58
    bottom = 16
    total_w = sum(im.width for im in resized) + gap * 4
    canvas = Image.new("RGB", (total_w, target_h + top + bottom), "white")
    draw = ImageDraw.Draw(canvas)
    x = gap
    for label, im in zip(labels, resized):
        draw.text((x + im.width // 2 - 35, 18), label, fill=(31, 41, 55))
        canvas.paste(im, (x, top))
        x += im.width + gap
    output.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(output)


def text_run(text: str, size: int = 22, bold: bool = False, color: str = "222222") -> str:
    b = "<a:b/>" if bold else ""
    return (
        f'<a:r><a:rPr lang="zh-CN" sz="{size * 100}" dirty="0">{b}'
        f'<a:solidFill><a:srgbClr val="{color}"/></a:solidFill>'
        f'<a:latin typeface="Microsoft YaHei"/><a:ea typeface="Microsoft YaHei"/>'
        f"</a:rPr><a:t>{esc(text)}</a:t></a:r>"
    )


def textbox(
    shape_id: int,
    x: int,
    y: int,
    w: int,
    h: int,
    paragraphs: list[str],
    size: int = 22,
    bold: bool = False,
    color: str = "222222",
    name: str = "TextBox",
    bullet: bool = False,
) -> str:
    ps = []
    for p in paragraphs:
        bullet_xml = '<a:buChar char="•"/>' if bullet else "<a:buNone/>"
        ps.append(
            f"<a:p><a:pPr marL=\"{emu(0.18) if bullet else 0}\" indent=\"{emu(-0.12) if bullet else 0}\">"
            f"{bullet_xml}</a:pPr>{text_run(p, size=size, bold=bold, color=color)}</a:p>"
        )
    return f"""
<p:sp>
  <p:nvSpPr><p:cNvPr id="{shape_id}" name="{name}"/><p:cNvSpPr txBox="1"/><p:nvPr/></p:nvSpPr>
  <p:spPr><a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{w}" cy="{h}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom><a:noFill/><a:ln><a:noFill/></a:ln></p:spPr>
  <p:txBody><a:bodyPr wrap="square" lIns="0" tIns="0" rIns="0" bIns="0"/><a:lstStyle/>{''.join(ps)}</p:txBody>
</p:sp>"""


def rect(
    shape_id: int,
    x: int,
    y: int,
    w: int,
    h: int,
    text: str,
    fill: str = "F3F6FA",
    line: str = "D7DEE8",
    size: int = 18,
    bold: bool = False,
    color: str = "222222",
    radius: str = "roundRect",
) -> str:
    return f"""
<p:sp>
  <p:nvSpPr><p:cNvPr id="{shape_id}" name="Box {shape_id}"/><p:cNvSpPr/><p:nvPr/></p:nvSpPr>
  <p:spPr><a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{w}" cy="{h}"/></a:xfrm>
    <a:prstGeom prst="{radius}"><a:avLst/></a:prstGeom>
    <a:solidFill><a:srgbClr val="{fill}"/></a:solidFill>
    <a:ln w="12700"><a:solidFill><a:srgbClr val="{line}"/></a:solidFill></a:ln>
  </p:spPr>
  <p:txBody><a:bodyPr anchor="ctr" wrap="square" lIns="91440" tIns="45720" rIns="91440" bIns="45720"/><a:lstStyle/>
    <a:p><a:pPr algn="ctr"/>{text_run(text, size=size, bold=bold, color=color)}</a:p>
  </p:txBody>
</p:sp>"""


def line(shape_id: int, x1: int, y1: int, x2: int, y2: int, color: str = "6B7280") -> str:
    return f"""
<p:cxnSp>
  <p:nvCxnSpPr><p:cNvPr id="{shape_id}" name="Connector {shape_id}"/><p:cNvCxnSpPr/><p:nvPr/></p:nvCxnSpPr>
  <p:spPr><a:xfrm><a:off x="{min(x1, x2)}" y="{min(y1, y2)}"/><a:ext cx="{abs(x2-x1)}" cy="{abs(y2-y1)}"/></a:xfrm>
    <a:prstGeom prst="line"><a:avLst/></a:prstGeom>
    <a:ln w="19050"><a:solidFill><a:srgbClr val="{color}"/></a:solidFill><a:tailEnd type="triangle"/></a:ln>
  </p:spPr>
</p:cxnSp>"""


def image(shape_id: int, rid: str, x: int, y: int, w: int, h: int, name: str) -> str:
    return f"""
<p:pic>
  <p:nvPicPr><p:cNvPr id="{shape_id}" name="{esc(name)}"/><p:cNvPicPr><a:picLocks noChangeAspect="1"/></p:cNvPicPr><p:nvPr/></p:nvPicPr>
  <p:blipFill><a:blip r:embed="{rid}"/><a:stretch><a:fillRect/></a:stretch></p:blipFill>
  <p:spPr><a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{w}" cy="{h}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></p:spPr>
</p:pic>"""


def hline(shape_id: int, x: int, y: int, w: int, color: str = "111827", width: int = 19050) -> str:
    return f"""
<p:sp>
  <p:nvSpPr><p:cNvPr id="{shape_id}" name="HLine {shape_id}"/><p:cNvSpPr/><p:nvPr/></p:nvSpPr>
  <p:spPr><a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{w}" cy="0"/></a:xfrm>
    <a:prstGeom prst="line"><a:avLst/></a:prstGeom>
    <a:ln w="{width}"><a:solidFill><a:srgbClr val="{color}"/></a:solidFill></a:ln>
  </p:spPr>
  <p:txBody><a:bodyPr/><a:lstStyle/><a:p/></p:txBody>
</p:sp>"""


def cell_text(shape_id: int, x: int, y: int, w: int, h: int, text: str,
              size: int = 15, bold: bool = False, color: str = "222222") -> str:
    return f"""
<p:sp>
  <p:nvSpPr><p:cNvPr id="{shape_id}" name="Cell {shape_id}"/><p:cNvSpPr txBox="1"/><p:nvPr/></p:nvSpPr>
  <p:spPr><a:xfrm><a:off x="{x}" y="{y}"/><a:ext cx="{w}" cy="{h}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom><a:noFill/><a:ln><a:noFill/></a:ln></p:spPr>
  <p:txBody><a:bodyPr anchor="ctr" wrap="square" lIns="0" tIns="0" rIns="0" bIns="0"/><a:lstStyle/>
    <a:p><a:pPr algn="ctr"/>{text_run(text, size=size, bold=bold, color=color)}</a:p>
  </p:txBody>
</p:sp>"""


def title_bar(title: str, subtitle: str | None = None) -> str:
    parts = [
        textbox(10, emu(0.55), emu(0.35), emu(12.2), emu(0.45), [title], size=27, bold=True),
        f'<p:sp><p:nvSpPr><p:cNvPr id="11" name="Title line"/><p:cNvSpPr/><p:nvPr/></p:nvSpPr><p:spPr><a:xfrm><a:off x="{emu(0.55)}" y="{emu(0.9)}"/><a:ext cx="{emu(12.2)}" cy="{emu(0.02)}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom><a:solidFill><a:srgbClr val="D9DEE7"/></a:solidFill><a:ln><a:noFill/></a:ln></p:spPr><p:txBody><a:bodyPr/><a:lstStyle/><a:p/></p:txBody></p:sp>',
    ]
    if subtitle:
        parts.append(textbox(12, emu(0.58), emu(0.97), emu(11.8), emu(0.35), [subtitle], size=13, color="666666"))
    return "".join(parts)


def slide_xml(content: str) -> str:
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sld xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
       xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
       xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:cSld><p:bg><p:bgPr><a:solidFill><a:srgbClr val="FFFFFF"/></a:solidFill><a:effectLst/></p:bgPr></p:bg><p:spTree>
    <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
    <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
    {content}
  </p:spTree></p:cSld><p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>
</p:sld>'''


def notes_xml(note: str) -> str:
    paragraphs = []
    for line_txt in note.splitlines():
        line_txt = line_txt.strip()
        if not line_txt:
            paragraphs.append("<a:p/>")
        else:
            paragraphs.append(f"<a:p>{text_run(line_txt, size=14, color='222222')}</a:p>")
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:notes xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
         xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"
         xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
  <p:cSld><p:spTree>
    <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
    <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
    <p:sp>
      <p:nvSpPr><p:cNvPr id="2" name="Notes Placeholder"/><p:cNvSpPr txBox="1"/><p:nvPr><p:ph type="body" idx="1"/></p:nvPr></p:nvSpPr>
      <p:spPr><a:xfrm><a:off x="685800" y="685800"/><a:ext cx="5486400" cy="7543800"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom><a:noFill/><a:ln><a:noFill/></a:ln></p:spPr>
      <p:txBody><a:bodyPr wrap="square"/><a:lstStyle/>{''.join(paragraphs)}</p:txBody>
    </p:sp>
  </p:spTree></p:cSld><p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>
</p:notes>'''


def write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def make_slide_rels(slide_no: int, rels: list[tuple[str, str]]) -> None:
    body = [
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
        '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">',
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideLayout" Target="../slideLayouts/slideLayout1.xml"/>',
    ]
    for rid, target in rels:
        body.append(
            f'<Relationship Id="{rid}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="{target}"/>'
        )
    body.append(
        f'<Relationship Id="rIdNotes" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/notesSlide" Target="../notesSlides/notesSlide{slide_no}.xml"/>'
    )
    body.append("</Relationships>")
    write(TMP / f"ppt/slides/_rels/slide{slide_no}.xml.rels", "\n".join(body))


def make_notes_rels(slide_no: int) -> None:
    write(TMP / f"ppt/notesSlides/_rels/notesSlide{slide_no}.xml.rels", f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slide" Target="../slides/slide{slide_no}.xml"/>
<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/notesMaster" Target="../notesMasters/notesMaster1.xml"/>
</Relationships>''')


def build() -> None:
    metrics = read_metrics()
    make_window_example(WINDOW_EXAMPLE)
    if TMP.exists():
        shutil.rmtree(TMP)
    (TMP / "ppt/media").mkdir(parents=True)

    split_map_candidates = (
        ROOT / "doc/fastlio.png",
        ROOT / "doc/liosam.png",
        ROOT / "doc/adaptive.png",
    )
    split_maps_available = all(p.exists() for p in split_map_candidates)

    images = {
        "traj": RESULTS / "evo_trajectory_latest_comparison_xy.png",
        "window_example": WINDOW_EXAMPLE,
    }
    if split_maps_available:
        images["fastlio_map"] = split_map_candidates[0]
        images["liosam_map"] = split_map_candidates[1]
        images["ours_map"] = split_map_candidates[2]
    for name, src in images.items():
        shutil.copyfile(src, TMP / f"ppt/media/{name}.png")

    slides: list[tuple[str, list[tuple[str, str]]]] = []

    slides.append((
        slide_xml(
            title_bar("3. 退化状态转换机制", "由单帧退化判断扩展为 Normal / Transient / Persistent 状态机")
            + rect(20, emu(0.8), emu(1.55), emu(2.75), emu(0.95), "Normal\n正常模式", fill="F8FAFC", line="CBD5E1", size=18, bold=True)
            + rect(21, emu(5.25), emu(1.55), emu(2.75), emu(0.95), "Transient\n短时退化", fill="FEF3C7", line="F59E0B", size=18, bold=True)
            + rect(22, emu(9.7), emu(1.55), emu(2.75), emu(0.95), "Persistent\n持续退化", fill="DBEAFE", line="3B82F6", size=18, bold=True)
            + line(23, emu(3.55), emu(2.02), emu(5.25), emu(2.02), color="6B7280")
            + line(24, emu(8.0), emu(2.02), emu(9.7), emu(2.02), color="6B7280")
            + line(25, emu(9.7), emu(2.78), emu(8.0), emu(2.78), color="94A3B8")
            + line(26, emu(5.25), emu(2.78), emu(3.55), emu(2.78), color="94A3B8")
            + textbox(27, emu(0.95), emu(3.35), emu(3.3), emu(1.85), [
                "当前帧未表现出退化风险。",
                "地图点按普通体素冗余逻辑和基础质量门控入图。",
            ], size=17, bullet=True)
            + textbox(28, emu(4.95), emu(3.35), emu(3.55), emu(1.85), [
                "当前帧被判定为退化，但窗口还没有确认持续退化。",
                "开始采用更谨慎的候选点处理，避免单帧噪声直接触发强限制。",
            ], size=17, bullet=True)
            + textbox(29, emu(9.0), emu(3.35), emu(3.55), emu(1.85), [
                "滑动窗口连续确认退化趋势后进入。",
                "地图更新进入更保守策略：候选点排序、novel 配额收紧、法向方向配额收紧。",
            ], size=17, bullet=True)
            + rect(30, emu(1.0), emu(5.75), emu(11.25), emu(0.68), "状态切换不是一帧触发：进入 Persistent 需要连续确认；退出也需要连续恢复，用滞回避免模式抖动。", fill="FFFFFF", line="94A3B8", size=17, bold=True)
        ),
        [],
    ))

    slides.append((
        slide_xml(
            title_bar("1. 当前模块流程图", "相对上次：在单帧退化判断之后，新增时间窗口判断持续退化")
            + rect(20, emu(0.65), emu(1.35), emu(2.25), emu(0.9), "h_share_model()\n残差/法向/有效点", fill="F8FAFC", line="CBD5E1", size=15, bold=True)
            + line(21, emu(2.9), emu(1.8), emu(3.45), emu(1.8))
            + rect(22, emu(3.45), emu(1.35), emu(2.15), emu(0.9), "单帧退化判断\nframe_degenerate", fill="FEF3C7", line="F59E0B", size=15, bold=True)
            + line(23, emu(5.6), emu(1.8), emu(6.15), emu(1.8))
            + rect(24, emu(6.15), emu(1.18), emu(2.45), emu(1.24), "新增\n滑动窗口", fill="DBEAFE", line="3B82F6", size=20, bold=True)
            + line(25, emu(8.6), emu(1.8), emu(9.15), emu(1.8))
            + rect(26, emu(9.15), emu(1.35), emu(1.55), emu(0.9), "退化模式\nMode", fill="EFF6FF", line="818CF8", size=15, bold=True)
            + line(27, emu(10.7), emu(1.8), emu(11.25), emu(1.8))
            + rect(28, emu(11.25), emu(1.35), emu(1.45), emu(0.9), "入图控制", fill="F0FDF4", line="22C55E", size=15, bold=True)
            + rect(29, emu(0.9), emu(3.0), emu(3.4), emu(1.25), "上次已有\n点级质量评价 + 帧级退化判断", fill="FFFFFF", line="CBD5E1", size=18, bold=True)
            + rect(30, emu(4.95), emu(2.85), emu(3.45), emu(1.55), "本次更新\n由“当前帧是否退化”扩展到\n“一段时间是否持续退化”", fill="EFF6FF", line="60A5FA", size=18, bold=True)
            + rect(31, emu(9.0), emu(3.0), emu(3.4), emu(1.25), "最终作用\nallow_map_insert_point()\n控制 ikd-tree 地图更新", fill="F0FDF4", line="86EFAC", size=17, bold=True)
            + textbox(32, emu(0.95), emu(5.15), emu(11.4), emu(0.7), [
                "一句话：h_share_model 提供观测质量，滑动窗口判断持续退化状态，地图插入函数根据状态选择更保守的入图策略。",
            ], size=18, color="374151")
        ),
        [],
    ))

    slides.append((
        slide_xml(
            title_bar("2. 滑动窗口本身如何判断持续退化？", "窗口用于把单帧退化提升为时间段退化，而不是再次做点云质量过滤")
            + image(20, "rId2", emu(0.75), emu(1.25), emu(11.85), emu(1.75), "Sliding window example")
            + textbox(21, emu(0.85), emu(3.28), emu(5.55), emu(1.95), [
                "图中每个格子表示一帧：N 为正常帧，D 为单帧退化结果。",
                "滑动窗口只保存最近一段时间；新帧从右侧进入，最旧帧从左侧移出。",
                "窗口输入来自 is_current_frame_degenerate() 和当前帧统计量，不直接读取原始点云。",
            ], size=17, bullet=True)
            + textbox(22, emu(6.8), emu(3.28), emu(5.45), emu(1.95), [
                "当窗口内多数帧退化，且最近几帧连续退化，同时机器人仍在直行前进，就认为更像长走廊/隧道式持续退化。",
                "这时先进入持续退化候选；只有连续多次确认后，状态才从 Transient 升级为 Persistent。",
            ], size=17, bullet=True)
            + rect(23, emu(1.0), emu(5.75), emu(11.25), emu(0.68), "关键点：滑动窗口负责判断“场景状态”，真正的点是否入图仍由 allow_map_insert_point() 执行。", fill="F0FDF4", line="86EFAC", size=17, bold=True, color="166534")
        ),
        [("rId2", "../media/window_example.png")],
    ))

    slides.append((
        slide_xml(
            title_bar("4. 判定为持续退化后怎么做？", "Persistent 不改变滤波器，而是让地图更新更保守")
            + rect(20, emu(0.7), emu(1.25), emu(2.35), emu(0.75), "窗口确认\nPersistent", fill="DBEAFE", line="3B82F6", size=17, bold=True)
            + line(21, emu(3.05), emu(1.62), emu(3.65), emu(1.62))
            + rect(22, emu(3.65), emu(1.25), emu(2.45), emu(0.75), "候选点排序", fill="EFF6FF", line="818CF8", size=17, bold=True)
            + line(23, emu(6.1), emu(1.62), emu(6.7), emu(1.62))
            + rect(24, emu(6.7), emu(1.25), emu(2.45), emu(0.75), "收紧入图配额", fill="FEF3C7", line="F59E0B", size=17, bold=True)
            + line(25, emu(9.15), emu(1.62), emu(9.75), emu(1.62))
            + rect(26, emu(9.75), emu(1.25), emu(2.45), emu(0.75), "更新 ikd-tree", fill="F0FDF4", line="22C55E", size=17, bold=True)
            + textbox(27, emu(0.85), emu(2.55), emu(3.4), emu(2.45), [
                "滑动窗口只输出退化模式。",
                "当模式为 Persistent，说明当前不是偶然退化，而是长时间处于走廊/隧道式弱约束环境。",
            ], size=17, bullet=True)
            + textbox(28, emu(4.55), emu(2.55), emu(3.55), emu(2.45), [
                "候选点不再只按普通顺序处理。",
                "Persistent 下优先保留对弱约束方向贡献更大的点，其次看质量分数。",
            ], size=17, bullet=True)
            + textbox(29, emu(8.45), emu(2.55), emu(3.75), emu(2.45), [
                "novel points 配额减小：允许地图继续生长，但降低不确定新点写入速度。",
                "法向方向配额减小：限制同一墙面/地面方向的重复点大量入图。",
            ], size=17, bullet=True)
            + rect(30, emu(0.95), emu(5.55), emu(11.45), emu(0.95), "最终执行位置：allow_map_insert_point() 决定每个点是否允许进入 point_to_add / point_no_need_downsample，随后由 p_map->Add_Points() 写入 ikd-tree。", fill="FFFFFF", line="94A3B8", size=16)
        ),
        [],
    ))

    if split_maps_available:
        exp_slide = (
            slide_xml(
                title_bar("5. 实验图像对比", "地图结果图 + 原轨迹对比图")
                + rect(20, emu(0.45), emu(1.16), emu(3.15), emu(0.35), "FAST-LIO", fill="F8FAFC", line="CBD5E1", size=14, bold=True)
                + rect(21, emu(3.85), emu(1.16), emu(3.15), emu(0.35), "LIO-SAM", fill="F8FAFC", line="CBD5E1", size=14, bold=True)
                + rect(22, emu(7.25), emu(1.16), emu(3.15), emu(0.35), "Ours", fill="EFF6FF", line="60A5FA", size=14, bold=True)
                + image(23, "rId2", emu(0.45), emu(1.58), emu(3.15), emu(2.15), "FAST-LIO map")
                + image(24, "rId3", emu(3.85), emu(1.58), emu(3.15), emu(2.15), "LIO-SAM map")
                + image(25, "rId4", emu(7.25), emu(1.58), emu(3.15), emu(2.15), "Ours map")
                + textbox(26, emu(10.65), emu(1.45), emu(2.15), emu(2.1), [
                    "FAST-LIO 漂移明显；LIO-SAM 较稳定；Ours 主走廊与转弯段更连续。",
                ], size=15, color="374151")
                + image(27, "rId5", emu(0.9), emu(4.15), emu(5.6), emu(2.05), "Trajectory comparison")
                + textbox(28, emu(6.8), emu(4.35), emu(5.5), emu(1.55), [
                    "轨迹对比图保留原 evo 结果：用于说明全局轨迹误差；地图结果图用于直观展示建图稳定性。",
                ], size=16, bullet=True)
            ),
            [
                ("rId2", "../media/fastlio_map.png"),
                ("rId3", "../media/liosam_map.png"),
                ("rId4", "../media/ours_map.png"),
                ("rId5", "../media/traj.png"),
            ],
        )
    else:
        exp_slide = (
            slide_xml(
                title_bar("5. 实验设置与轨迹对比", "SubT MRS Hawkins Long Corridor；三方法统一使用 evo 对齐评估")
                + image(20, "rId2", emu(0.6), emu(1.25), emu(7.15), emu(5.55), "Trajectory comparison")
                + textbox(21, emu(8.05), emu(1.45), emu(4.65), emu(4.6), [
                    "数据集：SubT MRS Hawkins Long Corridor",
                    "对比方法：Adaptive FAST-LIO2、FAST-LIO、LIO-SAM",
                    "评估工具：evo",
                    "APE/ATE：evo_ape tum ... -a",
                    "RPE：evo_rpe tum ... -a -d 10 -u m",
                    "注：若要插入你发的三张结果图，请保存为 doc/map_comparison_fastlio_liosam_ours.png 后重新生成。",
                ], size=18, bullet=True)
            ),
            [("rId2", "../media/traj.png")],
        )
    slides.append(exp_slide)

    a_ad = metrics[("APE_ATE", "Adaptive_FAST_LIO2")]["rmse_m"]
    a_fl = metrics[("APE_ATE", "FAST_LIO")]["rmse_m"]
    a_ls = metrics[("APE_ATE", "LIO_SAM")]["rmse_m"]
    r_ad = metrics[("RPE_10m", "Adaptive_FAST_LIO2")]["rmse_m"]
    r_fl = metrics[("RPE_10m", "FAST_LIO")]["rmse_m"]
    r_ls = metrics[("RPE_10m", "LIO_SAM")]["rmse_m"]
    a_ad_mean = metrics[("APE_ATE", "Adaptive_FAST_LIO2")]["mean_m"]
    a_fl_mean = metrics[("APE_ATE", "FAST_LIO")]["mean_m"]
    a_ls_mean = metrics[("APE_ATE", "LIO_SAM")]["mean_m"]
    r_ad_mean = metrics[("RPE_10m", "Adaptive_FAST_LIO2")]["mean_m"]
    r_fl_mean = metrics[("RPE_10m", "FAST_LIO")]["mean_m"]
    r_ls_mean = metrics[("RPE_10m", "LIO_SAM")]["mean_m"]

    slides.append((
        slide_xml(
            title_bar("6. 定量结果与结论", "采用论文常用三线表形式展示 ATE 与 10m RPE")
            + hline(20, emu(1.05), emu(1.55), emu(11.2), width=25400)
            + cell_text(21, emu(1.1), emu(1.66), emu(2.7), emu(0.45), "Method", size=16, bold=True)
            + cell_text(22, emu(3.8), emu(1.66), emu(2.1), emu(0.45), "ATE RMSE ↓", size=16, bold=True)
            + cell_text(23, emu(5.9), emu(1.66), emu(2.1), emu(0.45), "ATE Mean ↓", size=16, bold=True)
            + cell_text(24, emu(8.0), emu(1.66), emu(2.1), emu(0.45), "RPE 10m RMSE ↓", size=16, bold=True)
            + cell_text(25, emu(10.1), emu(1.66), emu(2.1), emu(0.45), "RPE 10m Mean ↓", size=16, bold=True)
            + hline(26, emu(1.05), emu(2.22), emu(11.2), width=19050)
            + cell_text(27, emu(1.1), emu(2.42), emu(2.7), emu(0.45), "Adaptive FAST-LIO2", size=15, bold=True, color="15803D")
            + cell_text(28, emu(3.8), emu(2.42), emu(2.1), emu(0.45), f"{a_ad:.3f} m", size=15, bold=True, color="15803D")
            + cell_text(29, emu(5.9), emu(2.42), emu(2.1), emu(0.45), f"{a_ad_mean:.3f} m", size=15, bold=True, color="15803D")
            + cell_text(30, emu(8.0), emu(2.42), emu(2.1), emu(0.45), f"{r_ad:.3f} m", size=15, bold=True, color="15803D")
            + cell_text(31, emu(10.1), emu(2.42), emu(2.1), emu(0.45), f"{r_ad_mean:.3f} m", size=15, bold=True, color="15803D")
            + cell_text(32, emu(1.1), emu(3.05), emu(2.7), emu(0.45), "FAST-LIO", size=15)
            + cell_text(33, emu(3.8), emu(3.05), emu(2.1), emu(0.45), f"{a_fl:.3f} m", size=15)
            + cell_text(34, emu(5.9), emu(3.05), emu(2.1), emu(0.45), f"{a_fl_mean:.3f} m", size=15)
            + cell_text(35, emu(8.0), emu(3.05), emu(2.1), emu(0.45), f"{r_fl:.3f} m", size=15)
            + cell_text(36, emu(10.1), emu(3.05), emu(2.1), emu(0.45), f"{r_fl_mean:.3f} m", size=15)
            + cell_text(37, emu(1.1), emu(3.68), emu(2.7), emu(0.45), "LIO-SAM", size=15)
            + cell_text(38, emu(3.8), emu(3.68), emu(2.1), emu(0.45), f"{a_ls:.3f} m", size=15)
            + cell_text(39, emu(5.9), emu(3.68), emu(2.1), emu(0.45), f"{a_ls_mean:.3f} m", size=15)
            + cell_text(40, emu(8.0), emu(3.68), emu(2.1), emu(0.45), f"{r_ls:.3f} m", size=15)
            + cell_text(41, emu(10.1), emu(3.68), emu(2.1), emu(0.45), f"{r_ls_mean:.3f} m", size=15)
            + hline(42, emu(1.05), emu(4.35), emu(11.2), width=25400)
            + textbox(43, emu(1.15), emu(4.8), emu(11.0), emu(1.35), [
                "结果说明：Adaptive FAST-LIO2 在全局 ATE 与局部 10m RPE 上均取得最小误差。",
                "原因分析：持续退化时，模块限制低价值或重复方向点进入地图，减少错误地图更新对后续匹配的牵引。",
            ], size=18, bullet=True)
        ),
        [],
    ))

    notes = [
        """这一页先讲本次新增的状态转换机制。
上次汇报里，模块主要是点级质量评价和单帧退化判断；这次新增的是把单帧结果放进滑动窗口，进一步区分短时退化和持续退化。
这里有三个状态：Normal 表示当前没有明显退化，地图更新基本按普通逻辑执行；Transient 表示当前帧退化，但还不能说明场景一直退化，所以只做相对轻的收紧；Persistent 表示窗口连续确认退化趋势，认为当前更像长走廊或隧道这种持续弱约束环境。
需要强调的是，状态切换不是一帧触发。一帧退化只会先进入 Transient，只有窗口连续确认后才进入 Persistent；退出也不是一帧恢复就退出，而是连续恢复后再回到普通状态。
这样做的目的，是避免因为单帧噪声或者短时拐角导致地图更新策略频繁抖动。""",
        """这一页是当前模块流程图。
最左边的 h_share_model 是系统前一步，它已经完成 scan-to-map 匹配，并提供点到面残差、法向量、有效点数量、Jacobian 条件数等信息。
上次已经讲过，我们基于这些信息做单点质量评价和单帧退化判断。
本次新增的是中间蓝色的滑动窗口：它把当前帧的退化结果和统计量放进最近一段时间的窗口里，判断是不是持续退化。
窗口输出的是退化模式，例如 Normal、Transient、Persistent。
最后真正执行地图更新控制的是 allow_map_insert_point，它决定点是否进入 point_to_add，然后由 p_map->Add_Points 写入 ikd-tree。""",
        """这一页讲滑动窗口的逻辑。
图中每个格子代表一帧，N 是正常帧，D 是这一帧被单帧逻辑判断为退化。
窗口只保存最近一段时间的帧，新帧从右边进入，最旧帧从左边移出。
窗口不是直接吃原始点云，而是吃 is_current_frame_degenerate 的结果，以及 h_share_model 后得到的统计量。
判断持续退化时，不只看窗口里退化帧多不多，还会看最近是否连续退化、机器人是否在持续前进、yaw 变化是否小、法向覆盖是否长期集中、残差是否稳定、条件数是否偏大。
直观理解就是：如果机器人在长走廊里一直往前走，视角变化不大，同时几何约束方向长期单一，那么这比单帧退化更可信，可以认为是持续退化候选。
但它不会一帧就切换模式，而是通过进入和退出计数做滞回，避免模式来回抖动。""",
        """这一页讲判定为持续退化之后到底怎么做。
首先要强调，Persistent 不改变 ESIKF 滤波器，也不是停止建图。
它的作用是在地图更新阶段更保守，也就是控制哪些点可以进入 ikd-tree。
第一步是候选点排序。普通退化时主要看质量分数；Persistent 时优先保留对弱约束方向贡献更大的点，其次再看质量分数。
第二步是收紧 novel points 的配额。novel points 是地图未知区域的新点，完全拒绝会导致地图不能向前生长，但全部接受又容易把不确定点写进去，所以 Persistent 下只允许更少的新点进入。
第三步是收紧法向方向配额。长走廊里很多点来自同一类墙面或地面，方向高度重复，继续大量写入对缺失自由度帮助不大，反而可能强化错误约束。
最终所有这些限制都在 allow_map_insert_point 里执行，允许的点才会进入 Add_Points 更新地图。""",
        """这一页是实验图像对比。
上面三张是实际建图结果，从左到右分别是 FAST-LIO、LIO-SAM 和 Ours。
FAST-LIO 在这个长走廊数据上漂移比较明显，能看到地图结构有重影和偏移。
LIO-SAM 整体比 FAST-LIO 稳定，但在结构连续性和细节上没有我们的结果完整。
Ours 的主走廊结构和右侧转弯段更连续，说明在长时间弱约束环境里，地图更新没有被错误点持续带偏。
下面保留原来的 evo 轨迹对比图，它用于从轨迹层面验证这个现象。
所以这一页想表达的是：地图形态上和轨迹误差上，滑动窗口后的版本都更稳定。""",
        """最后是定量结果。
这里用 evo 的 APE/ATE 和 10m RPE 进行对比，表格用论文常见的三线表形式。
ATE 反映全局轨迹误差，RPE 10m 反映局部十米尺度的相对运动误差。
从结果看，Adaptive FAST-LIO2 的 ATE RMSE 是 1.431 米，低于 LIO-SAM 的 1.730 米，也明显低于 FAST-LIO 的 18.144 米。
RPE 10m 上也是 Adaptive 最低，为 0.253 米，说明不仅全局不容易漂，局部运动一致性也更好。
结合前面的地图图像，可以说明滑动窗口持续退化判断和更保守的地图写入策略，对长走廊这种退化场景是有效的。
后续可以继续做的是：在更多长走廊、隧道和转角场景上验证，并进一步分析 Persistent 触发时刻与误差变化的对应关系。""",
    ]

    # 展示顺序：
    # 1. 当前模块流程图
    # 2. 滑动窗口判断逻辑
    # 3. 退化状态转换机制
    # 4. Persistent 后的地图更新策略
    # 5. 实验图像对比
    # 6. 定量结果与结论
    #
    # 代码中状态转换页先构造，是为了复用已有块；这里按汇报逻辑重排。
    order = [1, 2, 0, 3, 4, 5]
    slides = [slides[i] for i in order]
    notes = [notes[i] for i in order]

    for i, (xml, rels) in enumerate(slides, start=1):
        write(TMP / f"ppt/slides/slide{i}.xml", xml)
        make_slide_rels(i, rels)
        write(TMP / f"ppt/notesSlides/notesSlide{i}.xml", notes_xml(notes[i - 1]))
        make_notes_rels(i)

    notes_md = ["# PPT 演讲备注\n"]
    for i, note in enumerate(notes, start=1):
        notes_md.append(f"## 第 {i} 页\n\n{note}\n")
    write(RESULTS / "adaptive_fast_lio2_group_meeting_6slides_notes.md", "\n".join(notes_md))

    # Static package parts.
    content_types = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
                     '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">',
                     '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>',
                     '<Default Extension="xml" ContentType="application/xml"/>',
                     '<Default Extension="png" ContentType="image/png"/>',
                     '<Override PartName="/ppt/presentation.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.presentation.main+xml"/>',
                     '<Override PartName="/ppt/slideMasters/slideMaster1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideMaster+xml"/>',
                     '<Override PartName="/ppt/slideLayouts/slideLayout1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideLayout+xml"/>',
                     '<Override PartName="/ppt/notesMasters/notesMaster1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.notesMaster+xml"/>',
                     '<Override PartName="/ppt/theme/theme1.xml" ContentType="application/vnd.openxmlformats-officedocument.theme+xml"/>']
    for i in range(1, 7):
        content_types.append(f'<Override PartName="/ppt/slides/slide{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slide+xml"/>')
        content_types.append(f'<Override PartName="/ppt/notesSlides/notesSlide{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.notesSlide+xml"/>')
    content_types.append("</Types>")
    write(TMP / "[Content_Types].xml", "\n".join(content_types))

    write(TMP / "_rels/.rels", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="ppt/presentation.xml"/>
</Relationships>''')

    slide_ids = "\n".join([f'<p:sldId id="{255+i}" r:id="rId{i}"/>' for i in range(1, 7)])
    write(TMP / "ppt/presentation.xml", f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:presentation xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
<p:sldMasterIdLst><p:sldMasterId id="2147483648" r:id="rId7"/></p:sldMasterIdLst>
<p:sldIdLst>{slide_ids}</p:sldIdLst>
<p:sldSz cx="{SLIDE_W}" cy="{SLIDE_H}" type="wide"/>
<p:notesSz cx="6858000" cy="9144000"/>
<p:defaultTextStyle/>
</p:presentation>''')

    pres_rels = ['<?xml version="1.0" encoding="UTF-8" standalone="yes"?>',
                 '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">']
    for i in range(1, 7):
        pres_rels.append(f'<Relationship Id="rId{i}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slide" Target="slides/slide{i}.xml"/>')
    pres_rels.append('<Relationship Id="rId7" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="slideMasters/slideMaster1.xml"/>')
    pres_rels.append('</Relationships>')
    write(TMP / "ppt/_rels/presentation.xml.rels", "\n".join(pres_rels))

    write(TMP / "ppt/slideMasters/slideMaster1.xml", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sldMaster xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
<p:cSld><p:spTree><p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr><p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr></p:spTree></p:cSld>
<p:clrMap bg1="lt1" tx1="dk1" bg2="lt2" tx2="dk2" accent1="accent1" accent2="accent2" accent3="accent3" accent4="accent4" accent5="accent5" accent6="accent6" hlink="hlink" folHlink="folHlink"/>
<p:sldLayoutIdLst><p:sldLayoutId id="2147483649" r:id="rId1"/></p:sldLayoutIdLst><p:txStyles><p:titleStyle/><p:bodyStyle/><p:otherStyle/></p:txStyles>
</p:sldMaster>''')
    write(TMP / "ppt/slideMasters/_rels/slideMaster1.xml.rels", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideLayout" Target="../slideLayouts/slideLayout1.xml"/>
<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/theme" Target="../theme/theme1.xml"/>
</Relationships>''')
    write(TMP / "ppt/slideLayouts/slideLayout1.xml", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sldLayout xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main" type="blank" preserve="1">
<p:cSld name="Blank"><p:spTree><p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr><p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr></p:spTree></p:cSld><p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>
</p:sldLayout>''')
    write(TMP / "ppt/slideLayouts/_rels/slideLayout1.xml.rels", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/slideMaster" Target="../slideMasters/slideMaster1.xml"/>
</Relationships>''')

    write(TMP / "ppt/notesMasters/notesMaster1.xml", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:notesMaster xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" xmlns:p="http://schemas.openxmlformats.org/presentationml/2006/main">
<p:cSld><p:spTree><p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr><p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr></p:spTree></p:cSld>
<p:clrMap bg1="lt1" tx1="dk1" bg2="lt2" tx2="dk2" accent1="accent1" accent2="accent2" accent3="accent3" accent4="accent4" accent5="accent5" accent6="accent6" hlink="hlink" folHlink="folHlink"/>
<p:notesStyle/>
</p:notesMaster>''')
    write(TMP / "ppt/notesMasters/_rels/notesMaster1.xml.rels", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/theme" Target="../theme/theme1.xml"/>
</Relationships>''')

    write(TMP / "ppt/theme/theme1.xml", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<a:theme xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" name="Simple">
<a:themeElements><a:clrScheme name="Simple"><a:dk1><a:srgbClr val="000000"/></a:dk1><a:lt1><a:srgbClr val="FFFFFF"/></a:lt1><a:dk2><a:srgbClr val="1F2937"/></a:dk2><a:lt2><a:srgbClr val="F8FAFC"/></a:lt2><a:accent1><a:srgbClr val="2563EB"/></a:accent1><a:accent2><a:srgbClr val="16A34A"/></a:accent2><a:accent3><a:srgbClr val="DC2626"/></a:accent3><a:accent4><a:srgbClr val="F59E0B"/></a:accent4><a:accent5><a:srgbClr val="7C3AED"/></a:accent5><a:accent6><a:srgbClr val="0891B2"/></a:accent6><a:hlink><a:srgbClr val="2563EB"/></a:hlink><a:folHlink><a:srgbClr val="7C3AED"/></a:folHlink></a:clrScheme><a:fontScheme name="Simple"><a:majorFont><a:latin typeface="Microsoft YaHei"/><a:ea typeface="Microsoft YaHei"/></a:majorFont><a:minorFont><a:latin typeface="Microsoft YaHei"/><a:ea typeface="Microsoft YaHei"/></a:minorFont></a:fontScheme><a:fmtScheme name="Simple"><a:fillStyleLst/><a:lnStyleLst/><a:effectStyleLst/><a:bgFillStyleLst/></a:fmtScheme></a:themeElements>
</a:theme>''')

    if OUT.exists():
        OUT.unlink()
    with zipfile.ZipFile(OUT, "w", compression=zipfile.ZIP_DEFLATED) as z:
        for path in TMP.rglob("*"):
            if path.is_file():
                z.write(path, path.relative_to(TMP).as_posix())
    shutil.rmtree(TMP)
    print(OUT)


if __name__ == "__main__":
    build()
