"""Open the running dashboard, or start its existing interactive service."""
import json
import sys
from urllib.error import URLError
from urllib.request import ProxyHandler, build_opener
import webbrowser

import serve


def main():
    # Explicit command-line options keep the original serve.py behavior.
    if len(sys.argv) == 1:
        url = 'http://127.0.0.1:8765/'
        try:
            with build_opener(ProxyHandler({})).open(url + 'api/status', timeout=1) as reply:
                state = json.load(reply)
            if isinstance(state, dict) and {'connected', 'stop_latched', 'preparation', 'last_event_id'} <= state.keys():
                if not webbrowser.open(url):
                    print('Please open ' + url)
                return 0
        except (URLError, OSError, ValueError):
            pass
    return serve.main()


if __name__ == '__main__':
    raise SystemExit(main())
