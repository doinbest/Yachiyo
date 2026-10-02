# Yachiyo（八千代）

工训赛事搬运小车项目，包含 STM32F407VGT6 主车固件、香橙派视觉、串口屏、本地网页控制台和独立物料台转盘固件。

## 项目目录

```text
Yachiyo/
├── README.md
└── xjx/
    └── 模版搭建/
        ├── Code/
        ├── docs/
        ├── .embeddedskills/config.json
        ├── .gitignore
        ├── AGENTS.md
        └── README.md
```

[进入 xjx 的项目文件夹](xjx/模版搭建/) · [阅读项目说明](xjx/模版搭建/README.md) · [查看资料索引](xjx/模版搭建/docs/README.md)

## 使用入口

下载或克隆仓库后，进入 `xjx/模版搭建/` 作为项目工作目录。项目内部路径和工程配置保持原有结构。

- 主车 Keil 工程：`Code/template/MDK-ARM/template.uvprojx`，Target 为 `template`。
- 本地网页控制台：运行 `Code/upper-computer-web/local-car/start.cmd`。
- 香橙派视觉源码：`Code/orange_pi/vision.py`。
- 串口屏工程：`Code/HMI/display.HMI`。
- 独立物料台转盘工程：`Code/物料台转盘/turntable/MDK-ARM/turntable.uvprojx`。

详细硬件配置、操作步骤和当前功能范围以项目说明、协议文档和各模块 README 为准。

## 仓库内容

仓库保留项目源码、工程配置、文档、参考资料和已有提交历史。临时文件、编译输出、运行日志及编辑器个人设置按项目内的 `.gitignore` 排除。
