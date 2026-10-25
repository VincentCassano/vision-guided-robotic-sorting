"""Small JSON HTTP adapter. Backend owns all task-state decisions."""
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit


def create_server(host,port,backend):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):
            pass

        def reply(self,code,value):
            payload = json.dumps(value,allow_nan=False).encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type','application/json; charset=utf-8')
            self.send_header('Content-Length',str(len(payload)))
            self.send_header('Cache-Control','no-store')
            self.end_headers()
            self.wfile.write(payload)

        def dispatch(self):
            parsed = urlsplit(self.path)
            path = parsed.path
            if path not in ('/status','/start','/stop','/reset','/mode/manual','/mode/auto') or parsed.query:
                self.reply(404,{'error':'Unknown route'})
                return
            allowed = ('GET',) if path=='/status' else ('POST',) if path in ('/reset','/mode/manual','/mode/auto') else ('GET','POST')
            if self.command not in allowed:
                self.reply(405,{'error':'Method not allowed'})
                return
            try:
                length = int(self.headers.get('Content-Length','0'))
                if self.headers.get('Transfer-Encoding') or not 0 <= length <= 1024:
                    self.reply(413,{'error':'Unsupported or oversized body'})
                    return
                if length:
                    self.connection.settimeout(3)
                    body = json.loads(self.rfile.read(length).decode('utf-8'))
                    if body!={}:
                        self.reply(400,{'error':'Expected an empty JSON object'})
                        return
            except (ValueError,UnicodeError,TimeoutError,OSError):
                self.reply(400,{'error':'Invalid request body'})
                return
            try:
                result = backend(path[1:].replace('/','_').upper())
                self.reply(result.get('http_status',200),result)
            except Exception:
                self.reply(503,{'error':'Task controller unavailable'})

        do_GET = dispatch
        do_POST = dispatch
        do_PUT = dispatch
        do_DELETE = dispatch
        do_PATCH = dispatch

    return ThreadingHTTPServer((host,port),Handler)
