以后固定：

### Windows

- 改代码
    
- 提交 Git
    
- 编译
    
- 调试
    
- 串口
    
- MCP
    

### WSL

- 读代码
    
- 搜代码
    
- 跑 Codex
    
- 看日志
    
- 写建议
    
- 生成 patch 草稿

### 禁止：

- 不要在 WSL 和 Windows **同时**做编辑
    
- 不要在 WSL 里顺手提交 Git，再回 Windows 继续 IDE 提交
    
- 不要让两个环境都成为“主写入端”

目前WSL里有的：- `codex`
    
- `rg`
    
- `git`
    
- `node`








目前没有实现的：项目固定日志目录,项目建一个固定 AI 工作说明文件

在主项目根目录创建：

_debug_logs/

里面至少放：

_debug_logs/build_latest.txt  
_debug_logs/uart_latest.txt  
_debug_logs/issue_notes.md

用途：

- Windows 侧把构建输出、串口关键信息落盘
    
- WSL 侧 Codex 直接读这些文件分析
    
在项目根目录建：

AI_WORKFLOW.md

内容先简单写：

- 这个工程主编辑环境是 Windows IDE
    
- Git 提交只在 Windows 做
    
- WSL 只做阅读、搜索、AI 分析
    
- `_debug_logs` 是 AI 读日志入口
    
- 不允许 AI 直接批量重写核心驱动，除非你手动确认
    

这样你以后让 Codex 干活时，先让它读这个文件，能减少跑偏。
