# 八千代·巡航本地控制台

当前唯一控制台入口为 [local-car/README.md](local-car/README.md)。

Windows 双击 `local-car/start.cmd`，或运行 `python local-car/serve.py`，随后使用桌面 Chrome/Edge 访问 `http://127.0.0.1:8765/`。需要 Python 3 和 pyserial。

旧 React/Vinext 原型及其 `sys.ping#`、`motor.*#` 协议已退役，不再使用 `npm run dev`。本地控制台无需 npm 安装或云部署；Node.js 仅用于网页回归测试。

2026-09-21 清理仅涉及本地仓库，未操作远端站点。旧实现可通过 Git 历史追溯。
