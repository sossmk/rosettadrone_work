#!/usr/bin/env python3
"""RM500 ↔ PC 파일 다리. USB 데이터선이 안 통할 때 쓰는 우회로.

8/28 에 쓴 `python3 -m http.server` 는 PC→RM500 (내려받기) 만 됐다.
크래시 로그·버그신고 zip 을 가져오려면 반대 방향이 필요하다.

    python3 tools/file_bridge.py [포트=8000] [폴더=inbox]

RM500 브라우저에서  http://<PC IP>:8000/  를 연다.
  · 위쪽 = 올리기 (RM500 → PC).  받은 파일은 지정 폴더에 떨어진다
  · 아래쪽 = 내려받기 (PC → RM500). 현재 폴더의 파일 목록

⚠ 인증이 없다. 같은 핫스팟 안에서만 쓰고, 끝나면 Ctrl+C 로 끈다.
"""
import http.server, socketserver, os, sys, html, re

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
INBOX = sys.argv[2] if len(sys.argv) > 2 else "inbox"
os.makedirs(INBOX, exist_ok=True)

PAGE = """<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>RM500 파일 다리</title>
<style>body{{font:16px/1.6 sans-serif;margin:1.2em;max-width:40em}}
h2{{margin:1.4em 0 .4em;font-size:1.1em}}
input,button{{font-size:1.1em;padding:.5em}}
button{{background:#222;color:#fff;border:0;border-radius:4px}}
li{{margin:.3em 0}}</style>
<h2>① PC 로 올리기</h2>
<form method=post enctype=multipart/form-data>
  <input type=file name=f multiple><br><br>
  <button type=submit>올리기</button>
</form>
<p>{msg}</p>
<h2>② PC 에서 내려받기</h2>
<ul>{files}</ul>
"""


class Handler(http.server.SimpleHTTPRequestHandler):
    def _page(self, msg=""):
        try:
            names = sorted(os.listdir("."))
        except OSError:
            names = []
        items = "".join(
            f'<li><a href="{html.escape(n)}">{html.escape(n)}</a></li>'
            for n in names if os.path.isfile(n))
        body = PAGE.format(msg=html.escape(msg), files=items or "<li>(없음)</li>")
        b = body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            return self._page()
        return super().do_GET()

    def do_POST(self):
        """multipart/form-data 를 손으로 푼다.

        표준 cgi 모듈은 Python 3.13 에서 제거됐다. 이 도구는 새 PC 에서
        아무것도 설치하지 않고 돌아야 하므로 의존성을 만들지 않는다.
        """
        ctype = self.headers.get("Content-Type", "")
        m = re.search(r'boundary=(?:"([^"]+)"|([^;]+))', ctype)
        if not m:
            return self._page("✗ boundary 없음")
        boundary = (m.group(1) or m.group(2)).strip().encode()
        length = int(self.headers.get("Content-Length", 0))

        body = b""
        while len(body) < length:                 # 큰 파일도 다 읽는다
            chunk = self.rfile.read(min(1 << 20, length - len(body)))
            if not chunk:
                break
            body += chunk

        got = []
        for part in body.split(b"--" + boundary):
            if b"\r\n\r\n" not in part:
                continue
            head, data = part.split(b"\r\n\r\n", 1)
            fn = re.search(rb'filename="([^"]*)"', head)
            if not fn or not fn.group(1):
                continue
            data = data[:-2] if data.endswith(b"\r\n") else data   # 끝 CRLF 제거
            name = os.path.basename(fn.group(1).decode("utf-8", "replace"))
            dest = os.path.join(INBOX, name)
            with open(dest, "wb") as out:
                out.write(data)
            size = os.path.getsize(dest)
            got.append(f"{name} ({size:,} 바이트)")
            print(f"  \u2190 받음: {dest}  ({size:,} 바이트)")
        self._page("✓ 받았다: " + ", ".join(got) if got else "✗ 파일이 없다")


def my_ips():
    out = []
    for line in os.popen("ip -4 -o addr show scope global").read().splitlines():
        p = line.split()
        out.append((p[1], p[3].split("/")[0]))
    return out


socketserver.TCPServer.allow_reuse_address = True
with socketserver.ThreadingTCPServer(("0.0.0.0", PORT), Handler) as httpd:
    print(f"파일 다리 시작 — 받은 파일은 ./{INBOX}/ 에 떨어진다\n")
    for dev, ip in my_ips():
        mark = "  ← RM500 브라우저에 이 주소" if dev.startswith(("wl", "enx", "usb")) else ""
        print(f"    http://{ip}:{PORT}/   ({dev}){mark}")
    print("\nCtrl+C 로 종료")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n종료")
