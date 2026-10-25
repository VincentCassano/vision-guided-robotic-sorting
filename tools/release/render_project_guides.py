"""Render the two repository guides as local HTML documents, without project imports."""
import html
from pathlib import Path
import re

ROOT=Path(__file__).resolve().parents[2]
MOTTO='世上无难事，只要肯登攀。'


def inline(text):
    text=html.escape(text)
    text=re.sub(r'`([^`]+)`',r'<code>\1</code>',text)
    text=re.sub(r'\*\*([^*]+)\*\*',r'<strong>\1</strong>',text)
    def link(match):
        target=match.group(2)
        if target.startswith('docs/'):
            target=target[5:]
        elif not re.match(r'https?://',target):
            target='../'+target
        return '<a href="'+target+'">'+match.group(1)+'</a>'
    return re.sub(r'\[([^\]]+)\]\(([^)]+)\)',link,text)


def render(text):
    lines=text.splitlines();output=[];i=0
    while i<len(lines):
        line=lines[i]
        if not line.strip():i+=1;continue
        if line.startswith('```'):
            code=[];i+=1
            while i<len(lines) and not lines[i].startswith('```'):
                code.append(lines[i]);i+=1
            output.append('<pre><code>'+html.escape('\n'.join(code))+'</code></pre>');i+=1;continue
        if line.startswith('#'):
            level=len(line)-len(line.lstrip('#'))
            output.append('<h%d>%s</h%d>'%(level,inline(line[level:].strip()),level));i+=1;continue
        if line.startswith('|'):
            rows=[]
            while i<len(lines) and lines[i].startswith('|'):
                row=lines[i].strip().strip('|')
                if not re.fullmatch(r'[\s|:\-]+',row):rows.append([s.strip() for s in row.split('|')])
                i+=1
            output.append('<div class="table-wrap"><table>')
            for index,row in enumerate(rows):
                tag='th' if index==0 else 'td'
                output.append('<tr>'+''.join('<'+tag+'>'+inline(cell)+'</'+tag+'>' for cell in row)+'</tr>')
            output.append('</table></div>');continue
        if line.startswith('- '):
            output.append('<ul>')
            while i<len(lines) and lines[i].startswith('- '):
                output.append('<li>'+inline(lines[i][2:])+'</li>');i+=1
            output.append('</ul>');continue
        paragraph=[line];i+=1
        while i<len(lines) and lines[i].strip() and not lines[i].startswith(('#','```','|','- ')):
            paragraph.append(lines[i]);i+=1
        output.append('<p>'+inline(' '.join(paragraph))+'</p>')
    return '\n'.join(output)


STYLE='''
:root{color-scheme:light dark;--paper:#f8fafc;--ink:#243247;--line:#d9e1eb;--panel:#eef2f7}
*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);font:16px/1.8 system-ui,"Microsoft YaHei",sans-serif}
main{max-width:1040px;margin:auto;padding:48px 46px 64px}h1{font-size:32px;line-height:1.3;letter-spacing:-.6px;margin:0 0 18px}h2{font-size:23px;margin-top:42px;padding-bottom:9px;border-bottom:1px solid var(--line)}h3{font-size:18px;margin-top:26px}a{color:#3774c4;text-underline-offset:3px}p,ul{margin:16px 0}li{margin:7px 0}pre{padding:18px 20px;overflow:auto;background:var(--panel);border:1px solid var(--line);border-radius:10px;font:14px/1.65 ui-monospace,Consolas,monospace}code{font-size:.9em}p code,li code,td code{padding:2px 5px;background:var(--panel);border-radius:4px}.table-wrap{overflow:auto}table{border-collapse:collapse;width:100%;font-size:14px}th,td{border:1px solid var(--line);padding:11px 14px;text-align:left;vertical-align:top}th{background:var(--panel)}.selection-easter-egg{color:transparent;user-select:text;-webkit-user-select:text;margin-top:32px}.selection-easter-egg::selection{color:#102a43;background:#a5ceff}
@media(prefers-color-scheme:dark){:root{--paper:#101722;--ink:#e2e8f0;--line:#344155;--panel:#1a2535}a{color:#92bfff}}
@media(max-width:700px){main{padding:28px 20px}h1{font-size:26px}}
'''


def main():
    for source,name,lang in (('README.md','guide.zh-CN.html','zh-CN'),('README.en.md','guide.en.html','en')):
        page='<!doctype html>\n<html lang="'+lang+'"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
        page+='<title>Vision-Guided Robotic Grasping and Sorting System</title><style>'+STYLE+'</style><main>'
        page+=render((ROOT/source).read_text(encoding='utf8'))
        page+='<p class="selection-easter-egg">'+MOTTO+'</p></main></html>\n'
        (ROOT/'docs'/name).write_bytes(page.encode('utf8'))
    print('Wrote two HTML guides; no project/device execution.')


if __name__=='__main__':main()
