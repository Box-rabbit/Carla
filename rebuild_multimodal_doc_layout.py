from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
from datetime import datetime, timezone
from xml.sax.saxutils import escape


OUT = Path("多模态融合与场景适配说明.docx")


def esc(text: str) -> str:
    return escape(str(text))


def svg_rect(x, y, w, h, text_lines, fill="#F5F9FF", stroke="#4A6A8A"):
    lines = []
    lines.append(
        f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="12" ry="12" '
        f'fill="{fill}" stroke="{stroke}" stroke-width="2.5"/>'
    )
    font_y = y + h / 2 - (len(text_lines) - 1) * 16
    for i, line in enumerate(text_lines):
        lines.append(
            f'<text x="{x + w/2}" y="{font_y + i*32}" text-anchor="middle" '
            f'font-family="Microsoft YaHei, SimSun, Arial, sans-serif" font-size="24" '
            f'fill="#222222">{esc(line)}</text>'
        )
    return "".join(lines)


def svg_arrow(x1, y1, x2, y2):
    return (
        f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" '
        f'stroke="#555555" stroke-width="3.2" marker-end="url(#arrow)"/>'
    )


def svg_doc(width, height, body):
    return f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<defs>
  <marker id="arrow" markerWidth="12" markerHeight="12" refX="10" refY="6" orient="auto" markerUnits="strokeWidth">
    <path d="M0,0 L12,6 L0,12 z" fill="#555555"/>
  </marker>
</defs>
<rect x="0" y="0" width="{width}" height="{height}" fill="#FFFFFF"/>
{body}
</svg>'''


def build_svgs():
    svgs = {}

    body = []
    body.append(svg_rect(40, 170, 180, 90, ["离线语音样本", "data/audio"], fill="#FFF6EC", stroke="#C6803E"))
    body.append(svg_rect(280, 170, 190, 90, ["语音匹配配置", "route_audio_matches"], fill="#FFF6EC", stroke="#C6803E"))
    body.append(svg_rect(530, 170, 180, 90, ["语音触发模块", "RouteAudioRuntime"], fill="#FFF6EC", stroke="#C6803E"))
    body.append(svg_rect(770, 55, 200, 90, ["观测接口", "图像、LiDAR、车辆状态"], fill="#EEF7FF", stroke="#4A6A8A"))
    body.append(svg_rect(770, 285, 200, 90, ["路线与状态", "RouteTracker、场景状态"], fill="#EEF7FF", stroke="#4A6A8A"))
    body.append(svg_rect(1030, 170, 190, 90, ["控制输出", "油门、制动、转向"], fill="#EEF7FF", stroke="#4A6A8A"))
    body.append(svg_rect(1280, 170, 180, 90, ["日志记录", "frames.jsonl"], fill="#F2F7EF", stroke="#6B9555"))
    body.append(svg_arrow(220, 215, 280, 215))
    body.append(svg_arrow(470, 215, 530, 215))
    body.append(svg_arrow(710, 215, 770, 100))
    body.append(svg_arrow(710, 215, 770, 330))
    body.append(svg_arrow(970, 100, 1030, 215))
    body.append(svg_arrow(970, 330, 1030, 215))
    body.append(svg_arrow(1220, 215, 1280, 215))
    svgs["fig2_1.svg"] = svg_doc(1500, 430, "".join(body))

    body = []
    xs = [35, 250, 470, 690, 910, 1130, 1350]
    labels = [
        ["同步步进", "world.tick"],
        ["路线进度", "RouteTracker"],
        ["语音触发", "RouteAudioRuntime"],
        ["观测采集", "ObservationBuilder"],
        ["状态更新", "update_state"],
        ["控制计算", "compute_control"],
        ["逐帧记录", "frames.jsonl"],
    ]
    for x, lines in zip(xs, labels):
        body.append(svg_rect(x, 150, 150, 92, lines, fill="#F3FAF0", stroke="#678A58"))
    centers = [110, 325, 545, 765, 985, 1205, 1425]
    for a, b in zip(centers, centers[1:]):
        body.append(svg_arrow(a + 75, 196, b - 75, 196))
    svgs["fig2_2.svg"] = svg_doc(1540, 390, "".join(body))

    body = []
    xs = [20, 280, 540, 830, 1140, 1450]
    labels = [
        ["巡航"],
        ["转向"],
        ["左变道"],
        ["提速至80公里每小时"],
        ["减速至30公里每小时"],
        ["路线完成"],
    ]
    widths = [170, 170, 170, 240, 240, 170]
    x_now = 30
    centers = []
    for lines, w in zip(labels, widths):
        body.append(svg_rect(x_now, 150, w, 92, lines, fill="#FFF8EF", stroke="#C48848"))
        centers.append((x_now, w))
        x_now += w + 70
    for i in range(len(centers) - 1):
        x1 = centers[i][0] + centers[i][1]
        x2 = centers[i + 1][0]
        body.append(svg_arrow(x1, 196, x2, 196))
    svgs["fig3_1.svg"] = svg_doc(1650, 390, "".join(body))

    body = []
    labels = [
        ["巡航"],
        ["行人避让"],
        ["慢车跟随"],
        ["左变道超车"],
        ["返回原车道"],
        ["公交站谨慎通行"],
        ["恢复巡航"],
    ]
    widths = [150, 170, 170, 190, 180, 230, 170]
    x_now = 18
    centers = []
    for lines, w in zip(labels, widths):
        body.append(svg_rect(x_now, 150, w, 92, lines, fill="#EDF5FF", stroke="#54739B"))
        centers.append((x_now, w))
        x_now += w + 45
    for i in range(len(centers) - 1):
        x1 = centers[i][0] + centers[i][1]
        x2 = centers[i + 1][0]
        body.append(svg_arrow(x1, 196, x2, 196))
    svgs["fig4_1.svg"] = svg_doc(1600, 390, "".join(body))

    body = []
    labels = [
        ["巡航"],
        ["加塞监测"],
        ["紧急制动"],
        ["安全跟车"],
        ["施工并道"],
        ["施工区通过与回正"],
    ]
    widths = [150, 170, 170, 170, 170, 250]
    x_now = 30
    centers = []
    for lines, w in zip(labels, widths):
        body.append(svg_rect(x_now, 150, w, 92, lines, fill="#FFF2F2", stroke="#B16A6A"))
        centers.append((x_now, w))
        x_now += w + 60
    for i in range(len(centers) - 1):
        x1 = centers[i][0] + centers[i][1]
        x2 = centers[i + 1][0]
        body.append(svg_arrow(x1, 196, x2, 196))
    svgs["fig5_1.svg"] = svg_doc(1450, 390, "".join(body))

    body = []
    body.append(svg_rect(60, 145, 180, 92, ["正常状态"], fill="#F5F7FF", stroke="#6B78A8"))
    body.append(svg_rect(340, 145, 180, 92, ["谨慎状态"], fill="#F5F7FF", stroke="#6B78A8"))
    body.append(svg_rect(620, 145, 180, 92, ["紧急制动"], fill="#FDEFEF", stroke="#B16969"))
    body.append(svg_rect(900, 145, 180, 92, ["恢复状态"], fill="#F5F7FF", stroke="#6B78A8"))
    body.append(svg_rect(440, 305, 260, 82, ["失败事件记录"], fill="#FFF8E9", stroke="#B58A44"))
    body.append(svg_arrow(240, 191, 340, 191))
    body.append(svg_arrow(520, 191, 620, 191))
    body.append(svg_arrow(800, 191, 900, 191))
    body.append(svg_arrow(620, 237, 620, 305))
    svgs["fig6_1.svg"] = svg_doc(1140, 430, "".join(body))

    return svgs


class DocBuilder:
    def __init__(self):
        self.body = []
        self.image_rels = []
        self.rel_id = 10
        self.pic_id = 1

    def _r(self, text, mono=False, bold=False, size=None):
        rpr = []
        if mono or bold or size:
            rpr.append("<w:rPr>")
            if mono:
                rpr.append('<w:rFonts w:ascii="Courier New" w:hAnsi="Courier New" w:eastAsia="Courier New"/>')
            if bold:
                rpr.append("<w:b/>")
            if size:
                rpr.append(f'<w:sz w:val="{size}"/><w:szCs w:val="{size}"/>')
            rpr.append("</w:rPr>")
        return f'<w:r>{"".join(rpr)}<w:t xml:space="preserve">{esc(text)}</w:t></w:r>'

    def rich_p(self, parts, style=None, align=None, keep_next=False, page_break_before=False, first_line=420, spacing_before=0, spacing_after=120):
        ppr = ["<w:pPr>"]
        if style:
            ppr.append(f'<w:pStyle w:val="{style}"/>')
        if align:
            ppr.append(f'<w:jc w:val="{align}"/>')
        if keep_next:
            ppr.append("<w:keepNext/>")
        ppr.append("<w:keepLines/>")
        ppr.append("<w:widowControl/>")
        if page_break_before:
            ppr.append("<w:pageBreakBefore/>")
        if first_line:
            ppr.append(f'<w:ind w:firstLine="{first_line}"/>')
        ppr.append(f'<w:spacing w:before="{spacing_before}" w:after="{spacing_after}" w:line="420" w:lineRule="auto"/>')
        ppr.append("</w:pPr>")
        runs = []
        for item in parts:
            if isinstance(item, tuple):
                runs.append(self._r(item[0], mono=item[1]))
            else:
                runs.append(self._r(item))
        self.body.append(f'<w:p>{"".join(ppr)}{"".join(runs)}</w:p>')

    def plain_p(self, text, style=None, align=None, keep_next=False, page_break_before=False, first_line=420, spacing_before=0, spacing_after=120, bold=False):
        self.rich_p([(text, False)], style=style, align=align, keep_next=keep_next, page_break_before=page_break_before, first_line=first_line, spacing_before=spacing_before, spacing_after=spacing_after)

    def caption(self, text, is_table=False):
        self.body.append(
            f'<w:p><w:pPr><w:pStyle w:val="Caption"/><w:jc w:val="center"/><w:keepNext/><w:keepLines/><w:spacing w:before="60" w:after="120"/></w:pPr>{self._r(text, bold=False, size=20)}</w:p>'
        )

    def subhead(self, text):
        self.body.append(
            f'<w:p><w:pPr><w:pStyle w:val="Subhead"/><w:keepNext/><w:keepLines/><w:spacing w:before="140" w:after="80"/></w:pPr>{self._r(text, bold=True)}</w:p>'
        )

    def page_break(self):
        self.body.append('<w:p><w:r><w:br w:type="page"/></w:r></w:p>')

    def toc_field(self):
        self.body.append('<w:p><w:pPr><w:pStyle w:val="FrontHead"/></w:pPr><w:r><w:t>目录</w:t></w:r></w:p>')
        self.body.append(
            '<w:p>'
            '<w:r><w:fldChar w:fldCharType="begin"/></w:r>'
            '<w:r><w:instrText xml:space="preserve"> TOC \\o "1-2" \\h \\z \\u </w:instrText></w:r>'
            '<w:r><w:fldChar w:fldCharType="separate"/></w:r>'
            '<w:r><w:t></w:t></w:r>'
            '<w:r><w:fldChar w:fldCharType="end"/></w:r>'
            '</w:p>'
        )

    def table(self, title, rows, widths, code_cols=None):
        code_cols = set(code_cols or [])
        self.caption(title, is_table=True)
        tbl = [
            '<w:tbl>',
            '<w:tblPr>',
            '<w:tblStyle w:val="TableGrid"/>',
            '<w:tblW w:w="0" w:type="auto"/>',
            '<w:jc w:val="center"/>',
            '<w:tblLayout w:type="fixed"/>',
            '<w:tblCellMar><w:top w:w="70" w:type="dxa"/><w:left w:w="90" w:type="dxa"/><w:bottom w:w="70" w:type="dxa"/><w:right w:w="90" w:type="dxa"/></w:tblCellMar>',
            '<w:tblBorders><w:top w:val="single" w:sz="8" w:color="A6A6A6"/><w:left w:val="single" w:sz="8" w:color="A6A6A6"/><w:bottom w:val="single" w:sz="8" w:color="A6A6A6"/><w:right w:val="single" w:sz="8" w:color="A6A6A6"/><w:insideH w:val="single" w:sz="6" w:color="BFBFBF"/><w:insideV w:val="single" w:sz="6" w:color="BFBFBF"/></w:tblBorders>',
            '<w:tblLook w:firstRow="1" w:lastRow="0" w:firstColumn="0" w:lastColumn="0" w:noHBand="0" w:noVBand="1"/>',
            '</w:tblPr>',
        ]
        for r_index, row in enumerate(rows):
            header = r_index == 0
            tbl.append('<w:tr><w:trPr><w:cantSplit/>' + ('<w:tblHeader/>' if header else '') + '</w:trPr>')
            for c_index, cell in enumerate(row):
                shade = '<w:shd w:fill="D9E6F2"/>' if header else ''
                no_wrap = '<w:noWrap/>' if c_index in code_cols else ''
                tbl.append(
                    f'<w:tc><w:tcPr><w:tcW w:w="{widths[c_index]}" w:type="dxa"/>{shade}{no_wrap}<w:vAlign w:val="center"/></w:tcPr>'
                    f'<w:p><w:pPr><w:keepLines/><w:widowControl/><w:spacing w:before="50" w:after="50" w:line="300" w:lineRule="auto"/></w:pPr>'
                )
                mono = c_index in code_cols
                parts = str(cell).split("\n")
                for i, part in enumerate(parts):
                    if i > 0:
                        tbl.append('<w:r><w:br/></w:r>')
                    tbl.append(self._r(part, mono=mono, bold=header, size=18))
                tbl.append('</w:p></w:tc>')
            tbl.append('</w:tr>')
        tbl.append('</w:tbl>')
        self.body.append("".join(tbl))

    def image(self, name, content_type, data, width_in, height_in, caption):
        rid = f"rId{self.rel_id}"
        self.rel_id += 1
        target = f"media/{name}"
        self.image_rels.append((rid, target, data, content_type))
        cx = int(width_in * 914400)
        cy = int(height_in * 914400)
        pic_id = self.pic_id
        self.pic_id += 1
        self.body.append(
            f'''<w:p><w:pPr><w:jc w:val="center"/><w:keepNext/><w:keepLines/><w:spacing w:before="100" w:after="20"/></w:pPr><w:r><w:drawing>
<wp:inline xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing" distT="0" distB="0" distL="0" distR="0">
<wp:extent cx="{cx}" cy="{cy}"/><wp:docPr id="{pic_id}" name="{esc(name)}"/>
<a:graphic xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main">
<a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture">
<pic:pic xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture">
<pic:nvPicPr><pic:cNvPr id="0" name="{esc(name)}"/><pic:cNvPicPr/></pic:nvPicPr>
<pic:blipFill><a:blip r:embed="{rid}"/><a:stretch><a:fillRect/></a:stretch></pic:blipFill>
<pic:spPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="{cx}" cy="{cy}"/></a:xfrm><a:prstGeom prst="rect"><a:avLst/></a:prstGeom></pic:spPr>
</pic:pic></a:graphicData></a:graphic></wp:inline></w:drawing></w:r></w:p>'''
        )
        self.caption(caption)

    def landscape_section_break(self):
        self.body.append(
            '<w:p><w:pPr><w:sectPr>'
            '<w:pgSz w:w="11906" w:h="16838"/>'
            '<w:pgMar w:top="1440" w:right="1080" w:bottom="1440" w:left="1080" w:header="720" w:footer="720" w:gutter="0"/>'
            '<w:footerReference w:type="default" r:id="rId2"/>'
            '</w:sectPr></w:pPr></w:p>'
        )

    def portrait_section_break(self):
        self.body.append(
            '<w:p><w:pPr><w:sectPr>'
            '<w:pgSz w:w="11906" w:h="16838"/>'
            '<w:pgMar w:top="1440" w:right="1080" w:bottom="1440" w:left="1080" w:header="720" w:footer="720" w:gutter="0"/>'
            '<w:footerReference w:type="default" r:id="rId2"/>'
            '</w:sectPr></w:pPr></w:p>'
        )


def doc_styles():
    return '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
<w:style w:type="paragraph" w:default="1" w:styleId="Normal"><w:name w:val="Normal"/><w:pPr><w:widowControl/><w:spacing w:line="420" w:lineRule="auto"/></w:pPr><w:rPr><w:rFonts w:ascii="Times New Roman" w:hAnsi="Times New Roman" w:eastAsia="宋体"/><w:sz w:val="24"/><w:szCs w:val="24"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="CoverTitle"><w:name w:val="CoverTitle"/><w:basedOn w:val="Normal"/><w:pPr><w:jc w:val="center"/><w:spacing w:after="260"/></w:pPr><w:rPr><w:b/><w:rFonts w:eastAsia="黑体"/><w:sz w:val="36"/><w:szCs w:val="36"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="CoverLine"><w:name w:val="CoverLine"/><w:basedOn w:val="Normal"/><w:pPr><w:jc w:val="center"/><w:spacing w:after="80"/></w:pPr><w:rPr><w:sz w:val="24"/><w:szCs w:val="24"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="FrontHead"><w:name w:val="FrontHead"/><w:basedOn w:val="Normal"/><w:pPr><w:keepNext/><w:spacing w:before="120" w:after="120"/></w:pPr><w:rPr><w:b/><w:rFonts w:eastAsia="黑体"/><w:sz w:val="30"/><w:szCs w:val="30"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="Heading1"><w:name w:val="heading 1"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:pPr><w:outlineLvl w:val="0"/><w:pageBreakBefore/><w:keepNext/><w:keepLines/><w:spacing w:before="220" w:after="140"/></w:pPr><w:rPr><w:b/><w:rFonts w:eastAsia="黑体"/><w:sz w:val="30"/><w:szCs w:val="30"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="Heading2"><w:name w:val="heading 2"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:pPr><w:outlineLvl w:val="1"/><w:keepNext/><w:keepLines/><w:spacing w:before="160" w:after="100"/></w:pPr><w:rPr><w:b/><w:rFonts w:eastAsia="黑体"/><w:sz w:val="26"/><w:szCs w:val="26"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="Subhead"><w:name w:val="Subhead"/><w:basedOn w:val="Normal"/><w:next w:val="Normal"/><w:pPr><w:keepNext/><w:keepLines/><w:spacing w:before="140" w:after="80"/></w:pPr><w:rPr><w:b/><w:rFonts w:eastAsia="黑体"/><w:sz w:val="24"/><w:szCs w:val="24"/></w:rPr></w:style>
<w:style w:type="paragraph" w:styleId="Caption"><w:name w:val="Caption"/><w:basedOn w:val="Normal"/><w:pPr><w:jc w:val="center"/><w:keepNext/><w:keepLines/><w:spacing w:before="60" w:after="120"/></w:pPr><w:rPr><w:sz w:val="20"/><w:szCs w:val="20"/></w:rPr></w:style>
<w:style w:type="table" w:styleId="TableGrid"><w:name w:val="Table Grid"/></w:style>
</w:styles>'''


def settings_xml():
    return '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:settings xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
<w:updateFields w:val="true"/>
<w:displayBackgroundShape/>
</w:settings>'''


def footer_xml():
    return '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:ftr xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
<w:p><w:pPr><w:jc w:val="center"/></w:pPr><w:r><w:t>第</w:t></w:r><w:r><w:fldChar w:fldCharType="begin"/></w:r><w:r><w:instrText xml:space="preserve"> PAGE </w:instrText></w:r><w:r><w:fldChar w:fldCharType="separate"/></w:r><w:r><w:t>1</w:t></w:r><w:r><w:fldChar w:fldCharType="end"/></w:r><w:r><w:t>页</w:t></w:r></w:p>
</w:ftr>'''


def content_types(image_parts):
    defaults = [
        '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>',
        '<Default Extension="xml" ContentType="application/xml"/>',
        '<Default Extension="svg" ContentType="image/svg+xml"/>',
    ]
    overrides = [
        '<Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>',
        '<Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>',
        '<Override PartName="/word/footer1.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.footer+xml"/>',
        '<Override PartName="/word/settings.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.settings+xml"/>',
        '<Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>',
        '<Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>',
    ]
    return '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">' + "".join(defaults + overrides) + '</Types>'


def package_rels():
    return '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>
<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>
<Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>
</Relationships>'''


def document_rels(image_rels):
    rels = [
        '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>',
        '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/footer" Target="footer1.xml"/>',
        '<Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/settings" Target="settings.xml"/>',
    ]
    for rid, target, _, _ in image_rels:
        rels.append(f'<Relationship Id="{rid}" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/image" Target="{target}"/>')
    return '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">' + "".join(rels) + '</Relationships>'


def core_xml():
    now = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" xmlns:dc="http://purl.org/dc/elements/1.1/" xmlns:dcterms="http://purl.org/dc/terms/" xmlns:dcmitype="http://purl.org/dc/dcmitype/" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"><dc:title>多模态融合与场景适配说明</dc:title><dc:creator>Codex</dc:creator><cp:lastModifiedBy>Codex</cp:lastModifiedBy><dcterms:created xsi:type="dcterms:W3CDTF">{now}</dcterms:created><dcterms:modified xsi:type="dcterms:W3CDTF">{now}</dcterms:modified></cp:coreProperties>'''


def app_xml():
    return '<?xml version="1.0" encoding="UTF-8" standalone="yes"?><Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties" xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes"><Application>Codex OOXML Generator</Application></Properties>'


def document_xml(body):
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships" xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing" xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture">
<w:body>{body}
<w:sectPr>
<w:footerReference w:type="default" r:id="rId2"/>
<w:pgSz w:w="11906" w:h="16838"/>
<w:pgMar w:top="1440" w:right="1080" w:bottom="1440" w:left="1080" w:header="720" w:footer="720" w:gutter="0"/>
</w:sectPr>
</w:body></w:document>'''


def build_doc():
    doc = DocBuilder()

    doc.plain_p("多模态融合与场景适配说明", style="CoverTitle", align="center", first_line=0, spacing_before=2600, spacing_after=260)
    doc.plain_p("赛题编号：XH-202602", style="CoverLine", align="center", first_line=0)
    doc.plain_p("项目名称：面向智能驾驶的大模型应用场景研究", style="CoverLine", align="center", first_line=0)
    doc.plain_p("文档用途：基础赛道提交材料", style="CoverLine", align="center", first_line=0)
    doc.plain_p("团队名称：待补充", style="CoverLine", align="center", first_line=0)
    doc.plain_p("文档版本：V1.2", style="CoverLine", align="center", first_line=0)
    doc.plain_p("编写日期：2026年7月26日", style="CoverLine", align="center", first_line=0, spacing_after=1200)
    doc.page_break()

    doc.plain_p("修订记录", style="FrontHead", first_line=0, spacing_after=120)
    doc.table(
        "修订记录表",
        [
            ["版本", "日期", "修订内容", "依据"],
            ["V1.0", "2026年7月25日", "整理现有说明文本", "现有提交文档"],
            ["V1.1", "2026年7月26日", "按代码、配置和场景逻辑重写，锁定为六章结构", "carla_eval、configs、data/audio"],
            ["V1.2", "2026年7月26日", "优化目录、图表、分页、代码字体和表格版式", "当前文档排版调整"],
        ],
        widths=[1100, 1500, 4200, 2200],
    )
    doc.page_break()

    doc.toc_field()
    doc.page_break()

    doc.plain_p("1. 多模态输入组成", style="Heading1", first_line=0)
    doc.plain_p("1.1 语音指令", style="Heading2", first_line=0)
    doc.rich_p([
        "当前版本的语音输入来源是离线音频样本及其解析结果文件。样本文件位于",
        ("data/audio", True),
        "目录，匹配关系由",
        ("configs/lmdrive/route_audio_matches.yaml", True),
        "统一管理。每条样本包含",
        ("input_text", True),
        "、",
        ("normalized_text", True),
        "、",
        ("primary_intent", True),
        "、",
        ("all_intents", True),
        "、",
        ("confidence", True),
        "、",
        ("speed_values", True),
        "和",
        ("timing", True),
        "字段，匹配配置再将样本绑定到具体场景事件和触发距离。"
    ])
    doc.rich_p([
        "运行时语音入口在",
        ("RouteAudioRuntime.update", True),
        "中实现。该模块按照路线进度触发已匹配的语音事件，并把",
        ("voice_audio_id", True),
        "、",
        ("voice_event_id", True),
        "、",
        ("voice_text", True),
        "、",
        ("voice_intents", True),
        "、",
        ("voice_trigger_progress_m", True),
        "和",
        ("voice_trigger_timestamp", True),
        "写入逐帧日志。当前主评测链路没有接入在线语音识别、在线意图识别和实时槽位抽取，",
        ("Voice2LMDriveAdapter", True),
        "只保留外部适配接口。"
    ])
    doc.rich_p([
        "语音触发提前量由",
        ("voice_lead_distance_m", True),
        "配置为40.0米。S11、S12、S13的语音样本分别绑定到转向、变道、减速、公交站谨慎通行、突发加塞和施工并道等事件窗口。"
    ])

    doc.plain_p("1.2 前视/多视角图像", style="Heading2", first_line=0)
    doc.rich_p([
        "相机接入由",
        ("ObservationBuilder.SENSOR_SPECS", True),
        "定义。当前实际挂载的RGB相机包括前视、左视、右视和后视四路。前视相机分辨率为1200×900，视场角为100度；左、右、后视相机分辨率均为400×300，视场角均为100度。四路相机蓝图均为",
        ("sensor.camera.rgb", True),
        "。"
    ])
    doc.rich_p([
        "相机仅在",
        ("enable_cameras", True),
        "为",
        ("true", True),
        "时挂载。主评测脚本默认启用该接口，但当前三类场景状态机并未直接读取",
        ("rgb_front", True),
        "、",
        ("rgb_left", True),
        "、",
        ("rgb_right", True),
        "或",
        ("rgb_rear", True),
        "做在线感知决策，因此本文件将其定位为已接入的观测模态，而不是当前控制闭环中的直接判据。"
    ])

    doc.plain_p("1.3 LiDAR点云", style="Heading2", first_line=0)
    doc.rich_p([
        "LiDAR接口同样由",
        ("ObservationBuilder.SENSOR_SPECS", True),
        "定义，蓝图为",
        ("sensor.lidar.ray_cast", True),
        "，安装外参为",
        ("x=1.3", True),
        "、",
        ("y=0.0", True),
        "、",
        ("z=2.5", True),
        "、",
        ("yaw=-90.0", True),
        "。回调函数",
        ("_on_lidar", True),
        "把原始数据解析为",
        ("N×4", True),
        "数组，字段依次为",
        ("x", True),
        "、",
        ("y", True),
        "、",
        ("z", True),
        "和",
        ("intensity", True),
        "。"
    ])
    doc.rich_p([
        "当前代码没有显式覆盖LiDAR的",
        ("range", True),
        "、",
        ("channels", True),
        "、",
        ("rotation_frequency", True),
        "和",
        ("points_per_second", True),
        "，因此这些参数沿用CARLA蓝图默认值。当前三类场景状态机也没有直接从",
        ("lidar", True),
        "数组计算占据栅格或目标列表。"
    ])

    doc.plain_p("1.4 车辆状态", style="Heading2", first_line=0)
    doc.rich_p([
        "车辆状态是当前版本最核心的闭环输入。",
        ("evaluator.py", True),
        "在每个同步步采集自车位置、速度、转向、油门、制动、碰撞、车道侵入、红灯状态、路线进度、路线完成率和横向偏差。",
        ("RouteTracker.measure", True),
        "进一步把世界坐标映射到路线进度坐标，输出",
        ("route_progress_m", True),
        "、",
        ("route_completion", True),
        "、",
        ("lateral_offset_from_route_m", True),
        "和",
        ("route_deviation", True),
        "。"
    ])
    doc.rich_p([
        "S11的目标速度保持、转向完成和变道完成，S12的行人避让、超车和返回车道，S13的紧急制动、施工并道和通过施工区，都是通过车辆状态、路线进度和场景actor相对关系共同判定。"
    ])

    doc.plain_p("1.5 环境与天气信息", style="Heading2", first_line=0)
    doc.rich_p([
        "环境信息来自场景YAML、CARLA地图和场景actor配置。S11使用Town05和ClearNoon；S12使用Town05和CloudySunset；S13使用Town05和CustomHardRainNight。S13进一步显式配置了",
        ("cloudiness", True),
        "、",
        ("precipitation", True),
        "、",
        ("precipitation_deposits", True),
        "、",
        ("wetness", True),
        "、",
        ("wind_intensity", True),
        "、",
        ("fog_density", True),
        "、",
        ("fog_distance", True),
        "和",
        ("sun_altitude_angle", True),
        "。"
    ])
    doc.rich_p([
        "S13场景中存在可追溯的天气风险分数计算函数",
        ("_compute_hazard_score", True),
        "。该函数将",
        ("precipitation", True),
        "、",
        ("fog_density", True),
        "、夜间条件和",
        ("wetness", True),
        "组合为风险分数，用于极限应急场景的控制约束。"
    ])
    doc.table(
        "表1-1 多模态输入接口汇总",
        [
            ["模态", "实际来源", "更新方式或频率", "时间基准", "主要字段", "关键参数"],
            ["语音指令", "data/audio与route_audio_matches.yaml", "按路线进度触发一次事件", "frame×0.05秒", "voice_text、voice_intents、触发信息", "voice_lead_distance_m=40.0米"],
            ["前视图像", "CARLA sensor.camera.rgb", "同步世界下获取最新帧", "传感器回调", "rgb_front", "1200×900，FOV 100度"],
            ["左右后视图像", "CARLA sensor.camera.rgb", "同步世界下获取最新帧", "传感器回调", "rgb_left、rgb_right、rgb_rear", "400×300，FOV 100度"],
            ["LiDAR点云", "CARLA sensor.lidar.ray_cast", "同步世界下获取最新点云", "传感器回调", "lidar数组", "x=1.3，z=2.5，yaw=-90.0"],
            ["车辆状态", "ego actor、碰撞和车道追踪器", "每步更新", "frame×0.05秒", "速度、控制量、路线进度、偏差", "固定步长0.05秒"],
            ["环境与天气", "场景YAML、CARLA地图、场景actor", "场景初始化与状态机更新", "场景运行时", "天气参数、路线、交通灯、动作窗口", "Town05，场景自定义天气"],
        ],
        widths=[900, 1900, 1600, 1200, 2000, 1700],
        code_cols={1, 4},
    )
    doc.landscape_section_break()
    doc.table(
        "表1-2 实现依据",
        [
            ["代码文件", "类或函数", "配置文件", "关键字段"],
            ["carla_eval/lmdrive/\nroute_audio_runtime.py", "RouteAudioRuntime.update", "configs/lmdrive/\nroute_audio_matches.yaml", "audio_id、event_id、\ndistance_m"],
            ["carla_eval/sensors/\nobservation_builder.py", "ObservationBuilder\nSENSOR_SPECS", "运行参数\nenable_cameras", "rgb_front、rgb_left、\nrgb_right、rgb_rear、lidar"],
            ["carla_eval/\nevaluator.py", "ScenarioEvaluator.run", "三个场景YAML的\nruntime段", "timestamp、ego_speed_kmh、\nvoice字段"],
            ["carla_eval/\nruntime_metrics.py", "RouteTracker.measure", "三个场景YAML的\nevaluation段", "route_progress_m、\nroute_completion、\nlateral_offset_from_route_m"],
            ["configs/scenarios/\nemergency_response/\nS13_extreme_\nemergency_scene3_6km.yaml", "weather_parameters配置", "S13场景配置", "precipitation、wetness、\nfog_density、sun_altitude_angle"],
        ],
        widths=[2600, 1900, 2200, 2300],
        code_cols={0, 1, 2, 3},
    )
    doc.portrait_section_break()

    doc.plain_p("2. 语音语义到驾驶行为的对齐方式", style="Heading1", first_line=0)
    doc.subhead("时间对齐")
    doc.rich_p([
        "当前评测环境使用同步模式，",
        ("fixed_delta_seconds", True),
        "在S11、S12、S13三个场景中均配置为0.05秒。主循环时间戳按",
        ("frame×dt", True),
        "生成，图像、LiDAR点云和车辆状态都挂在同一个同步世界步长下更新。语音事件不是独立线程异步注入，而是在当前帧先根据",
        ("route_progress_m", True),
        "计算是否触发，再与本帧观测一起送入状态机。"
    ])
    doc.subhead("空间和路线进度对齐")
    doc.rich_p([
        "空间对齐的统一锚点是路线进度坐标。",
        ("RouteTracker.measure", True),
        "把自车世界坐标投影到路线折线，输出",
        ("route_progress_m", True),
        "、",
        ("route_completion", True),
        "、",
        ("lateral_offset_from_route_m", True),
        "和",
        ("route_deviation", True),
        "。内部连续搜索窗口默认在上一步进度后方20.0米和前方80.0米之间选择最优投影，从而减少重复路段和自交路段造成的跳变。"
    ])
    doc.rich_p([
        "路线偏离条件由",
        ("RouteTracker.measure", True),
        "统一判定：不在driving lane、横向偏差超过",
        ("route_corridor_half_width_m", True),
        "，或进度回退超过",
        ("backward_tolerance_m", True),
        "时标记",
        ("route_deviation", True),
        "。三类场景的走廊半宽分别为8.0米、8.5米和9.0米。"
    ])
    doc.subhead("语音语义结构化")
    doc.rich_p([
        "当前可追溯的语义结构化结果来自离线音频JSON和",
        ("route_audio_matches.yaml", True),
        "，而不是在线统一语义对象。离线样本中可直接读取",
        ("input_text", True),
        "、",
        ("corrected_text", True),
        "、",
        ("normalized_text", True),
        "、",
        ("primary_intent", True),
        "、",
        ("all_intents", True),
        "、",
        ("confidence", True),
        "、",
        ("speed_values", True),
        "和",
        ("timing", True),
        "。匹配配置再补充",
        ("event_id", True),
        "、",
        ("trigger.distance_m", True),
        "和",
        ("expected.target_speed_kmh", True),
        "。"
    ])
    doc.table(
        "表2-1 语义字段汇总",
        [
            ["字段", "含义", "来源"],
            ["input_text", "原始文本", "离线音频JSON"],
            ["corrected_text", "人工修正文本", "离线音频JSON"],
            ["normalized_text", "归一化文本", "离线音频JSON"],
            ["primary_intent", "主意图", "离线音频JSON"],
            ["all_intents", "候选意图集合", "离线音频JSON"],
            ["confidence", "离线置信度", "离线音频JSON"],
            ["speed_values", "文本中的速度槽位", "离线音频JSON"],
            ["event_id", "匹配后的场景事件", "route_audio_matches.yaml"],
            ["trigger.distance_m", "路线触发距离", "route_audio_matches.yaml"],
        ],
        widths=[2200, 3400, 3000],
        code_cols={0, 2},
    )
    doc.subhead("语义到驾驶子任务映射")
    doc.rich_p([
        "当前工程采用两层映射。第一层把语音样本映射到场景事件，例如S11的",
        ("right_turn_1", True),
        "、",
        ("lane_change_left", True),
        "和",
        ("accelerate_to_80", True),
        "，S12的",
        ("pedestrian_crossing", True),
        "、",
        ("slow_vehicle_overtake", True),
        "和",
        ("bus_stop_caution", True),
        "，S13的",
        ("sudden_cut_in_emergency", True),
        "和",
        ("construction_merge_left", True),
        "。第二层由场景YAML中的",
        ("instructions.expected_subtasks", True),
        "和",
        ("action_windows", True),
        "把事件映射为速度窗口、转向窗口、变道窗口和风险窗口。"
    ])
    doc.table(
        "表2-2 语义到子任务映射表",
        [
            ["语音指令", "语义结果", "场景事件", "驾驶子任务"],
            ["向右转弯。", "TURN_RIGHT", "right_turn_1", "进入右转窗口并按目标速度完成转向"],
            ["向左变道。", "LANE_CHANGE_LEFT", "lane_change_left", "在左变道窗口内跟踪横向偏移并保持"],
            ["前方公交站有行人上下车，注意减速到30公里每小时，确认安全后继续行驶。", "BUS_STOP_CAUTION、PEDESTRIAN_CAUTION、SLOW_DOWN", "bus_stop_caution", "公交站区域减速、谨慎通过并恢复巡航"],
            ["突发车辆加塞，紧急避让。", "CUT_IN_CAUTION、EMERGENCY_BRAKE", "sudden_cut_in_emergency", "触发前向距离和TTC风险监测"],
            ["施工路段减速并道至左侧车道。", "LANE_CHANGE_LEFT、AVOID_CONSTRUCTION、SLOW_DOWN", "construction_merge_left", "施工区前减速并道到左侧车道"],
        ],
        widths=[2600, 2500, 2100, 2200],
        code_cols={1, 2},
    )
    doc.subhead("模糊指令处理")
    doc.rich_p([
        "模糊指令不直接转换为单个油门或转角。当前版本通过离线匹配把模糊表达落到具体场景事件和窗口，例如“前方路况危险，保持安全车速”对应危险路段降速事件，“前方公交站有行人上下车，注意减速到30公里每小时，确认安全后继续行驶”对应公交站谨慎通行事件。事件触发后仍需满足状态机中的距离、速度、偏差和保持时间条件。"
    ])
    doc.subhead("低置信度输入处理")
    doc.rich_p([
        "离线样本中保留了",
        ("confidence", True),
        "和",
        ("low_confidence_intents", True),
        "字段。当前主评测链路没有单独的在线低置信度拒识阈值，因此本文件只把这部分能力描述为已保留输入字段；高风险动作是否执行仍由状态机的距离、TTC、横向偏差和速度条件决定。"
    ])
    doc.subhead("指令与场景匹配")
    doc.rich_p([
        "指令与场景匹配在",
        ("route_audio_matches.yaml", True),
        "中显式完成。运行时只加载",
        ("status", True),
        "为",
        ("matched", True),
        "且",
        ("scenario_id", True),
        "对应当前场景的条目。语音触发距离都以",
        ("route_progress_m", True),
        "为锚点，例如S11的右转样本在55.0米触发，S12的公交站样本在1720.0米触发，S13的加塞样本在1080.0米触发，施工并道样本在2480.0米触发。"
    ])
    doc.subhead("多信息源冲突裁决")
    doc.rich_p([
        "当前版本没有数值型动态模态权重。冲突裁决以安全优先级和场景阈值为主：路线偏离、红灯、碰撞、TTC、前向距离和横向偏差优先于语音意图。语音事件用于指定场景子任务和触发时机，但不覆盖安全约束。例如S13只有在前向距离不超过12.0米或TTC不超过1.2秒时才进入紧急制动阶段。"
    ])
    doc.subhead("高风险动作安全校验")
    doc.rich_p([
        "高风险动作的实际执行都附带二次安全校验。S11的左变道需要满足",
        ("completion_lateral_offset_m=0.8", True),
        "和保持1.0秒；S12的超车需要前向间距达到",
        ("lane_change_trigger_gap_m=15.0", True),
        "米并完成左车道偏移；S13的紧急制动需要满足",
        ("emergency_brake_distance_m=12.0", True),
        "米或",
        ("emergency_brake_ttc_s=1.2", True),
        "秒，施工并道需要达到",
        ("merge_lane_tolerance_m=0.9", True),
        "米并保持0.8秒。"
    ])
    doc.subhead("指标说明")
    doc.rich_p([
        "具体指标定义、测试过程、原始数据及量化结果详见《仿真测试全量报告》。"
    ])
    doc.landscape_section_break()
    doc.table(
        "表2-3 实现依据",
        [
            ["代码文件", "类或函数", "配置文件", "关键字段"],
            ["carla_eval/\nevaluator.py", "ScenarioEvaluator.run", "三个场景YAML的\nruntime段", "synchronous_mode、\nfixed_delta_seconds、\ntimestamp"],
            ["carla_eval/\nruntime_metrics.py", "RouteTracker.measure", "三个场景YAML的\nevaluation段", "route_progress_m、\nroute_completion、\nroute_deviation"],
            ["configs/lmdrive/\nroute_audio_matches.yaml", "语音匹配配置", "同文件", "scenario_id、event_id、\ntrigger.distance_m、\nexpected.target_speed_kmh"],
            ["data/audio目录下JSON", "离线语义样本", "同目录", "primary_intent、all_intents、\nconfidence、speed_values"],
            ["carla_eval/metrics/\nreport_generator.py", "infer_subtask_metrics", "场景YAML的\ninstructions段", "expected_subtasks"],
        ],
        widths=[2500, 1900, 2200, 2400],
        code_cols={0, 1, 2, 3},
    )
    doc.portrait_section_break()

    doc.plain_p("3. 基础操控场景适配", style="Heading1", first_line=0)
    doc.subhead("场景环境")
    doc.rich_p([
        "S11对应晴天白天城市道路连续驾驶。固定仿真步长为0.05秒，巡航目标速度为50.0公里每小时。场景要求车辆完成真实路线上的左右转、左变道、提速至80.0公里每小时和减速至30.0公里每小时。转向窗口支持自动按路线航向变化扫描，也保留显式配置窗口。"
    ])
    doc.subhead("指令及语义映射")
    doc.rich_p([
        "该场景绑定的离线指令包括“保持当前车道加速到80公里每小时”“注意前方向左转弯”“向右转弯”“向左变道”和“注意前方有行人横穿马路，注意减速”。这些指令分别映射到",
        ("accelerate_to_80", True),
        "、",
        ("left_turn_1", True),
        "、",
        ("right_turn_1", True),
        "、",
        ("lane_change_left", True),
        "和",
        ("slow_to_30", True),
        "。"
    ])
    doc.rich_p([
        "该语音样本在S11基础操控场景中仅用于触发减速至30公里每小时的纵向速度控制子任务，不承担行人检测、冲突区判断和行人避让功能。行人横穿避让逻辑在S12复杂避障场景中独立实现。"
    ])
    doc.subhead("状态机")
    doc.rich_p([
        "基础操控状态机从巡航开始，根据路线和动作窗口依次进入转向、变道、提速和减速阶段。控制器的基础前视距离为12.0米，转向前视距离为7.0米，变道前视距离为10.0米；基础转向增益为1.25，转向增益为1.55，变道增益为1.25；基础最大转角限制为0.46，转向为0.42，变道为0.38。"
    ])
    doc.subhead("触发条件")
    doc.rich_p([
        "右转窗口配置为95.0米至185.0米，目标速度为28.0公里每小时，最小航向变化阈值为35.0度。左转窗口配置为185.0米至305.0米，目标速度为30.0公里每小时，最小航向变化阈值为55.0度。左变道窗口配置为420.0米至730.0米；提速窗口配置为1500.0米至2500.0米；减速窗口配置为2900.0米至3500.0米。自动扫描转向窗口时，",
        ("turn_threshold_deg", True),
        "为35.0度，",
        ("turn_window_half_width_m", True),
        "为55.0米，",
        ("turn_min_gap_m", True),
        "为70.0米。"
    ])
    doc.subhead("决策动作")
    doc.rich_p([
        "加速阶段的目标速度为80.0公里每小时，",
        ("control_target_speed_kmh", True),
        "为95.0公里每小时，最小达到速度为78.0公里每小时，速度容差为7.0公里每小时。减速阶段的目标速度为30.0公里每小时，速度容差为4.0公里每小时。变道阶段跟踪",
        ("lateral_offset_profile", True),
        "生成的左侧路线中心，并通过",
        ("completion_lateral_offset_m=0.8", True),
        "判断横向切换到位。"
    ])
    doc.subhead("完成条件")
    doc.rich_p([
        "加速完成条件为速度达到不低于78.0公里每小时并保持1.0秒。减速完成条件为速度进入30.0公里每小时目标带并保持2.0秒。左变道完成条件为横向偏移达到配置阈值并保持1.0秒。长路线完成条件由",
        ("success_criteria.long_route", True),
        "给出，目标进度为5000.0米。"
    ])
    doc.subhead("中断和恢复")
    doc.rich_p([
        "该场景没有独立的恢复状态机。若在某一动作窗口内未满足完成条件，状态机会继续以当前窗口的目标速度和目标偏移运行，直到离开窗口或达到条件。路线偏离、红灯和车道侵入由评测层独立记录。"
    ])
    doc.subhead("异常处理")
    doc.rich_p([
        "基础操控阶段的异常处理以控制限幅和规则失败项为主。当转向幅度较大时，速度控制会主动压低油门；当",
        ("route_deviation", True),
        "、",
        ("collision", True),
        "、",
        ("lane_invasion", True),
        "、",
        ("red_light_violation", True),
        "、",
        ("blocked", True),
        "或",
        ("timeout", True),
        "发生时，由失败规则接管。"
    ])
    doc.table(
        "表3-1 基础操控场景决策逻辑表",
        [
            ["状态", "触发条件", "主要判断依据", "决策动作", "完成条件", "中断处理"],
            ["巡航", "默认状态", "路线目标点、当前速度", "保持50.0公里每小时巡航", "进入动作窗口", "按路线跟踪继续运行"],
            ["右转或左转", "进入转向窗口", "路线进度、航向变化阈值", "收缩前视距离并按28.0或30.0公里每小时转向", "达到最小航向变化阈值", "离开窗口后返回巡航"],
            ["左变道", "进入420.0至730.0米窗口", "路线进度、横向偏移", "跟踪左侧目标路线", "横向偏移达到0.8并保持1.0秒", "窗口结束后回到巡航逻辑"],
            ["提速至80", "进入1500.0至2500.0米窗口", "当前速度和保持时间", "按80.0公里每小时目标提速", "速度不低于78.0公里每小时并保持1.0秒", "未达成则持续提速直到离开窗口"],
            ["减速至30", "进入2900.0至3500.0米窗口", "当前速度和保持时间", "按30.0公里每小时目标减速", "进入容差带并保持2.0秒", "未达成则持续减速直到离开窗口"],
        ],
        widths=[900, 1700, 1700, 1900, 1700, 1500],
    )
    doc.landscape_section_break()
    doc.table(
        "表3-2 实现依据",
        [
            ["代码文件", "类或函数", "配置文件", "关键字段"],
            ["carla_eval/scenarios_impl/\ns11_basic_control_\nscene1.py", "initial_state\nupdate_state\ncompute_control", "configs/scenarios/\nbasic_control/\nS11_basic_control_\nscene1_5km.yaml", "active_window、\ntarget_speed_kmh、\ntarget_lateral_offset_m"],
            ["carla_eval/\nruntime_metrics.py", "RouteTracker.\npoint_at_progress_\nsmoothed", "同上", "route_progress_m、\nlateral_offset_from_route_m"],
            ["configs/lmdrive/\nroute_audio_matches.yaml", "S11语音事件配置", "同上", "right_turn_1、\nlane_change_left、\naccelerate_to_80、\nslow_to_30"],
        ],
        widths=[2500, 1800, 2200, 2400],
        code_cols={0, 1, 2, 3},
    )
    doc.portrait_section_break()

    doc.plain_p("4. 复杂避障场景适配", style="Heading1", first_line=0)
    doc.subhead("场景环境")
    doc.rich_p([
        "S12对应阴天傍晚条件下的复杂避障工况，固定仿真步长同样为0.05秒，巡航目标速度为50.0公里每小时。场景包含三段连续任务：行人横穿减速避让、慢车左变道超越并返回原车道、公交站区域减速谨慎通行。"
    ])
    doc.subhead("指令及语义映射")
    doc.rich_p([
        "该场景绑定三条语音样本，分别对应",
        ("pedestrian_crossing", True),
        "、",
        ("slow_vehicle_overtake", True),
        "和",
        ("bus_stop_caution", True),
        "。公交站语音样本在",
        ("speed_values", True),
        "中明确给出30.0公里每小时目标速度。"
    ])
    doc.subhead("状态机")
    doc.rich_p([
        "控制器在不同阶段使用不同前视距离和转向限制。巡航前视距离为12.0米；行人避让为8.0米；跟车为10.0米；超车为14.0米；回正为9.0米；公交站谨慎通行为11.0米。相应的转向增益分别为1.25、1.45、1.35、1.18、1.70和1.35；最大转角限制分别为0.46、0.40、0.36、0.34、0.46和0.34。"
    ])
    doc.subhead("触发条件")
    doc.rich_p([
        "行人横穿窗口为300.0米至480.0米，锚点为360.0米，检测距离阈值为24.0米，最小安全距离为4.5米，保持时间为1.0秒。慢车超越段的动作窗口为1050.0米至1280.0米；慢车速度为20.0公里每小时，检测间距阈值为30.0米，变道触发间距为15.0米。公交站窗口为1760.0米至1980.0米，锚点为1850.0米，目标速度为30.0公里每小时，速度容差为4.0公里每小时，保持时间为2.0秒。"
    ])
    doc.subhead("决策动作")
    doc.rich_p([
        "行人避让阶段优先制动，并在未达到安全条件前保持低速或停车。跟车阶段目标速度为30.0公里每小时。超车阶段目标速度为50.0公里每小时，目标横向偏移为-3.5米，左车道就绪判据为横向偏移不大于-2.6米。返回原车道阶段要求横向偏差不大于0.25米并保持2.0秒。"
    ])
    doc.subhead("完成条件")
    doc.rich_p([
        "行人避让完成条件为：行人完成横穿，车辆处于低速安全状态，最小安全距离满足4.5米，并保持1.0秒。超车完成条件为：车辆完成左侧超越并满足返回原车道条件。公交站谨慎通行完成条件为：车辆在公交站区域达到目标速度约束并安全通过。长路线完成目标进度为8000.0米。"
    ])
    doc.subhead("中断和恢复")
    doc.rich_p([
        "场景子任务按顺序串行推进。行人避让未完成前，慢车超越逻辑不会进入有效触发阶段；慢车超越未完成前，返回原车道和公交站谨慎通行不会进入完成判定。恢复动作主要体现在返回原车道阶段和公交站通过后的巡航恢复。"
    ])
    doc.subhead("异常处理")
    doc.rich_p([
        "该场景的异常处理围绕",
        ("collision", True),
        "、",
        ("route_deviation", True),
        "、",
        ("red_light_violation", True),
        "、",
        ("lane_invasion", True),
        "、",
        ("timeout", True),
        "和",
        ("blocked", True),
        "展开。当前状态机没有独立的后向高速来车感知模块，因此超车安全判断基于场景车辆间距、路线偏移和速度关系。"
    ])
    doc.table(
        "表4-1 复杂避障场景决策逻辑表",
        [
            ["状态", "触发条件", "主要判断依据", "决策动作", "完成条件", "中断处理"],
            ["行人避让", "进入300.0至480.0米窗口且行人距离不大于24.0米", "行人位置、自车速度、最小安全距离", "优先制动并维持低速", "行人清空冲突区且安全距离满足4.5米并保持1.0秒", "未完成则保持制动"],
            ["慢车跟随", "行人避让完成后慢车激活", "front gap、慢车速度", "按30.0公里每小时跟车", "front gap进入变道触发阈值", "间距过小时继续制动"],
            ["左变道超车", "front gap不大于15.0米", "横向偏移、速度关系、路线进度", "向左侧车道偏移并提速到50.0公里每小时", "完成超越并具备回正条件", "未完成则维持超车控制"],
            ["返回原车道", "超车完成后自动进入", "横向偏差和保持时间", "向原车道回正", "横向偏差不大于0.25米并保持2.0秒", "未完成则持续回正"],
            ["公交站谨慎通行", "进入1760.0至1980.0米窗口", "公交站锚点、行人actor、当前速度", "减速到30.0公里每小时并谨慎通过", "满足速度容差并完成通过", "未完成则维持谨慎通行"],
        ],
        widths=[900, 1700, 1700, 1900, 1700, 1500],
    )
    doc.landscape_section_break()
    doc.table(
        "表4-2 实现依据",
        [
            ["代码文件", "类或函数", "配置文件", "关键字段"],
            ["carla_eval/scenarios_impl/\ns12_complex_obstacle_\nscene2.py", "update_state\ncompute_control\nextra_record", "configs/scenarios/\ncomplex_obstacle/\nS12_complex_obstacle_\nscene2_8km.yaml", "pedestrian_detected、\nlane_change_started、\novertake_completed、\nbus_stop_pass_completed"],
            ["carla_eval/\nruntime_metrics.py", "RouteTracker\n路线目标点函数", "同上", "route_progress_m、\nlateral_offset_from_route_m"],
            ["configs/lmdrive/\nroute_audio_matches.yaml", "S12语音事件配置", "同上", "pedestrian_crossing、\nslow_vehicle_overtake、\nbus_stop_caution"],
        ],
        widths=[2500, 1800, 2200, 2400],
        code_cols={0, 1, 2, 3},
    )
    doc.portrait_section_break()

    doc.plain_p("5. 极限应急场景适配", style="Heading1", first_line=0)
    doc.subhead("场景环境")
    doc.rich_p([
        "S13对应雨夜低能见度下的极限应急工况，固定仿真步长为0.05秒，巡航目标速度为50.0公里每小时。天气参数中",
        ("precipitation", True),
        "为60.0，",
        ("precipitation_deposits", True),
        "为70.0，",
        ("wetness", True),
        "为85.0，",
        ("fog_density", True),
        "为12.0，",
        ("sun_altitude_angle", True),
        "为-14.0。核心任务包括突发车辆加塞应急制动和施工路段减速并道至左侧车道。"
    ])
    doc.subhead("指令及语义映射")
    doc.rich_p([
        "该场景绑定三条语音样本，分别对应危险路段安全车速、突发车辆加塞和施工并道。当前文档只把这些样本作为场景事件触发入口，高风险动作是否执行由状态机中的距离、TTC、横向偏移和保持时间条件决定。"
    ])
    doc.subhead("状态机")
    doc.rich_p([
        "S13控制器针对巡航、转向、偏离恢复、施工并道、紧急制动和施工工人阶段分别设置参数。巡航前视距离为14.0米，转向前视距离为7.0米，偏离恢复前视距离为6.5米，施工并道前视距离为12.0米，紧急制动前视距离为8.0米，施工工人阶段前视距离为8.0米。紧急制动最大转角限制为0.28，施工并道为0.36，偏离恢复为0.46。"
    ])
    doc.subhead("触发条件")
    doc.rich_p([
        "突发加塞段在ego进度达到1120.0米时进入激活区，NPC初始进度为1260.0米，初始横向偏移为-3.5米，",
        ("merge_delay_seconds", True),
        "为0.6秒，",
        ("merge_duration_seconds", True),
        "为2.8秒。加塞检测距离阈值为70.0米；紧急制动触发条件为前向距离不超过12.0米或TTC不超过1.2秒。施工区",
        ("detection_progress_m", True),
        "为2480.0米，",
        ("merge_start_progress_m", True),
        "为2520.0米，",
        ("zone_end_progress_m", True),
        "为2650.0米，",
        ("return_end_progress_m", True),
        "为2700.0米。"
    ])
    doc.subhead("决策动作")
    doc.rich_p([
        "突发加塞阶段一旦满足风险阈值，状态机会设置",
        ("emergency_brake_started", True),
        "，并进入以制动优先为主的控制策略。安全跟车目标速度为25.0公里每小时，安全跟车距离为16.0米，完成保持时间为0.8秒。施工并道阶段的目标横向偏移为-3.5米，横向容差为0.9米，保持时间为0.8秒。偏离恢复在横向偏差不小于1.0米时触发，目标速度限制为22.0公里每小时。"
    ])
    doc.rich_p([
        "控制限幅方面，",
        ("max_throttle_cap", True),
        "为0.8；当转角绝对值超过0.3时，",
        ("throttle", True),
        "进一步限制为0.18。转向阶段目标速度为26.0公里每小时；施工并道阶段目标速度为30.0公里每小时。"
    ])
    doc.subhead("完成条件")
    doc.rich_p([
        "紧急制动阶段的完成条件为：前向距离达到不小于16.0米、横向偏移回到允许范围内、车速不高于25.0公里每小时加1.0公里每小时容差，并保持0.8秒。施工并道完成条件为：车辆横向偏移进入左侧目标车道容差带，并保持0.8秒。长路线目标进度为5950.0米，目标路线完成率为0.99。"
    ])
    doc.subhead("中断和恢复")
    doc.rich_p([
        "紧急制动完成后，状态机会进入安全跟车和后续施工并道阶段。施工区通过后，车辆在",
        ("return_end_progress_m", True),
        "之前继续执行回归路线的控制。当前版本没有单独的最低风险停车状态机，恢复过程由场景状态机中的安全跟车、并道保持和路线回正逻辑完成。"
    ])
    doc.subhead("异常处理")
    doc.rich_p([
        "S13额外维护",
        ("ttc_s", True),
        "、",
        ("min_ttc", True),
        "、",
        ("front_vehicle_gap", True),
        "、",
        ("worker_distance_m", True),
        "和",
        ("construction_target_lateral_offset_m", True),
        "等字段。若加塞actor、施工工人或锥桶区域引发高风险状态，控制器优先降低速度、限制横向动作并维持路线约束。"
    ])
    doc.table(
        "表5-1 极限应急场景决策逻辑表",
        [
            ["状态", "触发条件", "主要判断依据", "决策动作", "完成条件", "中断处理"],
            ["突发加塞监测", "进度达到1120.0米激活区", "front gap、横向偏移、相对速度", "持续监测加塞车状态", "进入风险阈值判定", "若actor未正常激活则保持巡航约束"],
            ["紧急制动", "front gap不大于12.0米或TTC不大于1.2秒", "TTC、前向距离、速度", "优先制动，最大转角限制0.28", "安全跟车距离达到16.0米并保持0.8秒", "未达成则继续制动"],
            ["安全跟车", "紧急制动后自动进入", "front gap、横向偏移、当前速度", "保持不高于25.0公里每小时的安全跟车", "完成保持时间后退出", "若风险回升则重新进入紧急制动"],
            ["施工并道", "进度达到2520.0米并进入施工区", "目标横向偏移、车道容差、工人和锥桶位置", "减速并道到左侧车道", "横向容差不大于0.9米并保持0.8秒", "未并道完成则维持并道控制"],
            ["施工区通过与回正", "通过并道阶段后自动进入", "施工区终点、回正终点、路线偏差", "沿施工区安全通过并恢复路线中心", "超过施工区终点并完成回正", "若偏差过大则触发偏离恢复"],
        ],
        widths=[900, 1700, 1700, 1900, 1700, 1500],
    )
    doc.landscape_section_break()
    doc.table(
        "表5-2 实现依据",
        [
            ["代码文件", "类或函数", "配置文件", "关键字段"],
            ["carla_eval/scenarios_impl/\ns13_extreme_emergency_\nscene3.py", "_compute_hazard_score\nupdate_state\ncompute_control\nextra_record", "configs/scenarios/\nemergency_response/\nS13_extreme_emergency_\nscene3_6km.yaml", "ttc_s、\nemergency_brake_started、\nmerge_completed、\nworker_detected"],
            ["carla_eval/\nruntime_metrics.py", "RouteTracker\n路线目标点函数", "同上", "route_progress_m、\nlateral_offset_from_route_m"],
            ["configs/lmdrive/\nroute_audio_matches.yaml", "S13语音事件配置", "同上", "dangerous_road_\nsafety_speed、\nsudden_cut_in_\nemergency、\nconstruction_merge_left"],
        ],
        widths=[2500, 1800, 2200, 2400],
        code_cols={0, 1, 2, 3},
    )
    doc.portrait_section_break()

    doc.plain_p("6. 异常处理与安全兜底", style="Heading1", first_line=0)
    doc.subhead("语音异常")
    doc.rich_p([
        "当前主评测链路只接入离线语音事件。若没有匹配到当前场景的语音条目，",
        ("RouteAudioRuntime", True),
        "不会激活语音事件，车辆继续按场景默认指令和状态机运行。",
        ("Voice2LMDriveAdapter", True),
        "保留在线适配接口，但没有进入当前主流程，因此不在本文件中扩展在线异常分支。"
    ])
    doc.subhead("传感器异常")
    doc.rich_p([
        "图像和LiDAR点云的接入方式是获取最新帧或最新点云。当前代码没有单独的传感器超时计时器和数据失效阈值，安全兜底主要依赖车辆状态、路线进度、场景actor和交通规则。"
    ])
    doc.subhead("多信息源冲突")
    doc.rich_p([
        "当前版本的冲突处理不采用数值权重，而采用安全优先级裁决。路线偏离、红灯、碰撞、前向距离、TTC和横向偏差优先于语音事件和动作意图。高风险动作只能在对应场景条件成立时执行。"
    ])
    doc.subhead("路线偏离")
    doc.rich_p([
        "路线偏离由",
        ("RouteTracker.measure", True),
        "统一判断。若不在",
        ("driving lane", True),
        "、横向偏差超过场景走廊半宽，或进度异常回退，则",
        ("route_deviation", True),
        "置为",
        ("true", True),
        "。S13还配置了",
        ("deviation_recovery_trigger_m=1.0", True),
        "、",
        ("deviation_recovery_lookahead_m=6.5", True),
        "、",
        ("deviation_recovery_steer_gain=1.75", True),
        "和",
        ("deviation_recovery_speed_kmh=22.0", True),
        "，用于偏离恢复。"
    ])
    doc.subhead("控制异常")
    doc.rich_p([
        "控制兜底体现在速度控制、转向限幅和制动优先级中。S11在大转角时会抑制油门；S12在行人和跟车阶段可直接返回高制动值；S13限制最大油门为0.8，当转角绝对值大于0.3时把油门进一步限制到0.18。"
    ])
    doc.subhead("超时和阻塞")
    doc.rich_p([
        "三类场景都定义了",
        ("max_duration_seconds", True),
        "，并在",
        ("failure_criteria", True),
        "中保留",
        ("timeout", True),
        "和",
        ("blocked", True),
        "条件。超时和阻塞属于评测层失败项，场景控制器本身不额外扩展编号状态，而是继续执行当前最保守的可行控制。"
    ])
    doc.subhead("降级控制")
    doc.rich_p([
        "当前版本可归纳为四类降级控制：保持当前车道并继续路线跟踪；降低目标速度并延长谨慎通行阶段；限制横向动作和油门幅度；在紧急制动阶段优先制动并维持安全跟车。该抽象与本文中的正常状态、谨慎状态、紧急制动和恢复状态描述相对应，用于说明工程实现边界。"
    ])
    doc.subhead("恢复机制")
    doc.rich_p([
        "恢复机制由各场景的保持时间和退出条件实现，而不是独立恢复控制器。S11的加速、减速和变道都要求保持时间；S12的行人避让、返回车道和公交站谨慎通行都有保持时间；S13的安全跟车和施工并道同样使用0.8秒保持条件。条件满足后，状态机会回到巡航或下一阶段逻辑。"
    ])
    doc.subhead("日志记录")
    doc.rich_p([
        "逐帧日志由",
        ("evaluator.py", True),
        "写入",
        ("frames.jsonl", True),
        "，事件日志由",
        ("event_detector.py", True),
        "从帧日志提取，统计报告由",
        ("report_generator.py", True),
        "生成。当前日志中真实可见的安全相关字段包括",
        ("collision", True),
        "、",
        ("lane_invasion", True),
        "、",
        ("red_light_violation", True),
        "、",
        ("route_deviation", True),
        "、",
        ("route_progress_m", True),
        "、",
        ("lateral_offset_from_route_m", True),
        "、",
        ("ttc_s", True),
        "、",
        ("voice_trigger_timestamp", True),
        "以及各场景",
        ("extra_record", True),
        "输出的阶段字段。"
    ])
    doc.rich_p([
        "具体指标定义、测试过程、原始数据及量化结果详见《仿真测试全量报告》。"
    ])
    doc.landscape_section_break()
    doc.table(
        "表6-1 实现依据",
        [
            ["代码文件", "类或函数", "配置文件", "关键字段"],
            ["carla_eval/\nruntime_metrics.py", "RouteTracker.measure\nLaneInvasionTracker\nRedLightViolationTracker", "三个场景YAML的\nevaluation和\nfailure_criteria段", "route_deviation、\nlane_invasion、\nred_light_violation"],
            ["carla_eval/scenarios_impl/\ns11_basic_control_\nscene1.py", "compute_control", "S11场景配置", "大转角时油门抑制"],
            ["carla_eval/scenarios_impl/\ns12_complex_obstacle_\nscene2.py", "compute_control", "S12场景配置", "行人和跟车阶段的\n制动控制"],
            ["carla_eval/scenarios_impl/\ns13_extreme_emergency_\nscene3.py", "update_state\ncompute_control\nextra_record", "S13场景配置", "ttc_s、\nemergency_brake_started、\ndeviation_recovery、\nmax_throttle_cap"],
            ["carla_eval/metrics/\nevent_detector.py\ncarla_eval/metrics/\nreport_generator.py", "事件抽取和\n报告生成", "configs/metrics/\nmetric_schema.yaml", "frames.jsonl、events、\nevaluation_report"],
        ],
        widths=[2500, 1800, 2200, 2400],
        code_cols={0, 1, 2, 3},
    )
    doc.portrait_section_break()

    return doc


def write_docx():
    doc = build_doc()
    with ZipFile(OUT, "w", ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", content_types(doc.image_rels))
        z.writestr("_rels/.rels", package_rels())
        z.writestr("docProps/core.xml", core_xml())
        z.writestr("docProps/app.xml", app_xml())
        z.writestr("word/document.xml", document_xml("".join(doc.body)))
        z.writestr("word/styles.xml", doc_styles())
        z.writestr("word/settings.xml", settings_xml())
        z.writestr("word/footer1.xml", footer_xml())
        z.writestr("word/_rels/document.xml.rels", document_rels(doc.image_rels))
        for _, target, data, _ in doc.image_rels:
            z.writestr(f"word/{target}", data)


if __name__ == "__main__":
    write_docx()
