"use client";

import { useCallback, useEffect, useRef, useState } from "react";

type SerialPortLike = {
  open(options: { baudRate: number }): Promise<void>;
  close(): Promise<void>;
  readable: ReadableStream<Uint8Array> | null;
  writable: WritableStream<Uint8Array> | null;
};

type SerialNavigator = Navigator & {
  serial?: {
    requestPort(): Promise<SerialPortLike>;
  };
};

type LogEntry = { time: string; direction: "TX" | "RX" | "SYS"; text: string };

const readItems = [
  ["vel", "实时速度"], ["cpos", "实时位置"], ["perr", "位置误差"],
  ["vbus", "总线电压"], ["cbus", "总线电流"], ["cpha", "相电流"],
  ["flag", "电机标志"], ["oflag", "回零标志"],
];

function now() {
  return new Date().toLocaleTimeString("zh-CN", { hour12: false });
}

export default function Home() {
  const [connected, setConnected] = useState(false);
  const [baudRate, setBaudRate] = useState(115200);
  const [motorId, setMotorId] = useState(1);
  const [speed, setSpeed] = useState(300);
  const [acceleration, setAcceleration] = useState(20);
  const [pulses, setPulses] = useState(3200);
  const [sync, setSync] = useState(false);
  const [homeMode, setHomeMode] = useState(0);
  const [rawCommand, setRawCommand] = useState("sys.ping#");
  const [logs, setLogs] = useState<LogEntry[]>([
    { time: now(), direction: "SYS", text: "上位机已就绪，等待连接 Zigbee USB 串口。" },
  ]);
  const portRef = useRef<SerialPortLike | null>(null);
  const readerRef = useRef<ReadableStreamDefaultReader<Uint8Array> | null>(null);
  const readLoopRef = useRef<Promise<void> | null>(null);

  const addLog = useCallback((direction: LogEntry["direction"], text: string) => {
    setLogs((items) => [...items.slice(-159), { time: now(), direction, text }]);
  }, []);

  const readLoop = useCallback(async (port: SerialPortLike) => {
    const decoder = new TextDecoder();
    let pending = "";
    while (port.readable) {
      const reader = port.readable.getReader();
      readerRef.current = reader;
      try {
        while (true) {
          const { value, done } = await reader.read();
          if (done) break;
          pending += decoder.decode(value, { stream: true });
          const chunks = pending.split(/(?<=#)\r?\n?|\r?\n/);
          pending = chunks.pop() ?? "";
          chunks.filter(Boolean).forEach((line) => addLog("RX", line.trim()));
        }
      } catch (error) {
        if (connected) addLog("SYS", `读取停止：${String(error)}`);
      } finally {
        reader.releaseLock();
        readerRef.current = null;
      }
      break;
    }
  }, [addLog, connected]);

  const send = useCallback(async (command: string) => {
    const port = portRef.current;
    if (!connected || !port?.writable) {
      addLog("SYS", "请先连接串口。未发送任何指令。");
      return;
    }
    const framed = command.trim().endsWith("#") ? command.trim() : `${command.trim()}#`;
    const writer = port.writable.getWriter();
    try {
      await writer.write(new TextEncoder().encode(framed));
      addLog("TX", framed);
    } finally {
      writer.releaseLock();
    }
  }, [addLog, connected]);

  const connect = async () => {
    const serial = (navigator as SerialNavigator).serial;
    if (!serial) {
      addLog("SYS", "当前浏览器不支持 Web Serial，请使用桌面版 Chrome 或 Edge。 ");
      return;
    }
    try {
      const port = await serial.requestPort();
      await port.open({ baudRate });
      portRef.current = port;
      setConnected(true);
      addLog("SYS", `串口已连接：${baudRate} bps / 8N1`);
      readLoopRef.current = readLoop(port);
      window.setTimeout(async () => {
        if (!port.writable) return;
        const writer = port.writable.getWriter();
        try {
          await writer.write(new TextEncoder().encode("sys.ping#"));
          addLog("TX", "sys.ping#");
        } finally {
          writer.releaseLock();
        }
      }, 120);
    } catch (error) {
      addLog("SYS", `连接取消或失败：${String(error)}`);
    }
  };

  const disconnect = useCallback(async () => {
    const port = portRef.current;
    if (!port) return;
    try {
      await readerRef.current?.cancel();
      await readLoopRef.current;
      await port.close();
    } catch (error) {
      addLog("SYS", `断开时提示：${String(error)}`);
    } finally {
      portRef.current = null;
      setConnected(false);
      addLog("SYS", "串口已断开。 ");
    }
  }, [addLog]);

  useEffect(() => () => { void disconnect(); }, [disconnect]);

  const clamp = (value: number, min: number, max: number) =>
    Number.isFinite(value) ? Math.min(max, Math.max(min, Math.trunc(value))) : min;

  return (
    <main>
      <header className="topbar">
        <div className="brandMark">R</div>
        <div className="brandText"><strong>ROBOWALKER CONTROL</strong><span>PRECISION MOTION · ZIGBEE LINK</span></div>
        <div className={`connectionBadge ${connected ? "online" : ""}`}><i />{connected ? "设备在线" : "设备离线"}</div>
      </header>

      <section className="hero">
        <div><p className="eyebrow">STM32F407 · ZDT X42S</p><h1>步进电机控制台</h1><p>通过 Zigbee 透明串口连接 USART2，完成运动控制、回零与状态诊断。</p></div>
        <div className="deviceBar">
          <label>波特率<select value={baudRate} disabled={connected} onChange={(e) => setBaudRate(Number(e.target.value))}><option>115200</option><option>57600</option><option>38400</option><option>9600</option></select></label>
          <button className="primary" onClick={connected ? disconnect : connect}>{connected ? "断开设备" : "连接设备"}</button>
          <button className="ghost" onClick={() => send("sys.ping#")}>握手测试</button>
        </div>
      </section>

      <section className="layout">
        <div className="controlColumn">
          <article className="panel motorPanel">
            <div className="panelTitle"><div><span>运动控制</span><small>MOTION CONTROL</small></div><label className="idInput">电机 ID<input type="number" min="1" max="255" value={motorId} onChange={(e) => setMotorId(clamp(Number(e.target.value), 1, 255))} /></label></div>
            <div className="enableRow"><button className="enable" onClick={() => send(`motor.enable=${motorId},1#`)}>使能电机</button><button onClick={() => send(`motor.enable=${motorId},0#`)}>失能电机</button><button className="danger" onClick={() => send("motor.stop_all#")}>急停全部</button></div>
            <div className="modeGrid">
              <div className="modeCard">
                <div className="modeHead"><strong>速度模式</strong><span>±5000 rpm</span></div>
                <label>目标速度 <b>{speed} rpm</b><input type="range" min="-5000" max="5000" step="10" value={speed} onChange={(e) => setSpeed(Number(e.target.value))} /></label>
                <div className="inputPair"><label>速度<input type="number" min="-5000" max="5000" value={speed} onChange={(e) => setSpeed(clamp(Number(e.target.value), -5000, 5000))} /></label><label>加速度<input type="number" min="0" max="255" value={acceleration} onChange={(e) => setAcceleration(clamp(Number(e.target.value), 0, 255))} /></label></div>
                <div className="buttonRow"><button onClick={() => setSpeed(-Math.abs(speed || 300))}>CCW</button><button className="primary" onClick={() => send(`motor.speed=${motorId},${speed},${acceleration}#`)}>发送速度</button><button onClick={() => setSpeed(Math.abs(speed || 300))}>CW</button></div>
              </div>
              <div className="modeCard">
                <div className="modeHead"><strong>相对位置</strong><span>脉冲模式</span></div>
                <label>目标脉冲<input className="largeInput" type="number" min="-2147483648" max="2147483647" value={pulses} onChange={(e) => setPulses(clamp(Number(e.target.value), -2147483648, 2147483647))} /></label>
                <div className="inputPair"><label>速度 rpm<input type="number" min="0" max="5000" value={Math.abs(speed)} onChange={(e) => setSpeed(clamp(Number(e.target.value), 0, 5000))} /></label><label className="checkLabel"><input type="checkbox" checked={sync} onChange={(e) => setSync(e.target.checked)} />等待同步启动</label></div>
                <div className="buttonRow"><button onClick={() => send(`motor.move=${motorId},${-Math.abs(pulses)},${Math.abs(speed)},${acceleration},${sync ? 1 : 0}#`)}>反向移动</button><button className="primary" onClick={() => send(`motor.move=${motorId},${pulses},${Math.abs(speed)},${acceleration},${sync ? 1 : 0}#`)}>执行位置</button><button onClick={() => send("motor.sync#")}>同步启动</button></div>
              </div>
            </div>
            <button className="stopSingle" onClick={() => send(`motor.stop=${motorId}#`)}>停止当前电机</button>
          </article>

          <article className="panel utilityPanel">
            <div className="panelTitle"><div><span>基准与回零</span><small>REFERENCE &amp; HOMING</small></div></div>
            <div className="utilityGrid"><button onClick={() => send(`motor.zero=${motorId}#`)}>当前位置清零</button><label>回零模式<select value={homeMode} onChange={(e) => setHomeMode(Number(e.target.value))}><option value="0">单圈就近</option><option value="1">单圈定向</option><option value="2">多圈碰撞</option><option value="3">多圈限位</option></select></label><button className="primary" onClick={() => send(`motor.home=${motorId},${homeMode},0#`)}>开始回零</button><button className="warn" onClick={() => send(`motor.home_abort=${motorId}#`)}>中止回零</button></div>
            <p className="warningText">碰撞/限位回零必须先在电机端正确配置阈值与硬件，首次测试请脱离负载并准备急停。</p>
          </article>
        </div>

        <aside className="sideColumn">
          <article className="panel statusPanel">
            <div className="panelTitle"><div><span>状态读取</span><small>DIAGNOSTICS</small></div></div>
            <div className="readGrid">{readItems.map(([key, label]) => <button key={key} onClick={() => send(`motor.read=${motorId},${key}#`)}><span>{label}</span><code>{key}</code></button>)}</div>
            <button className="wide" onClick={() => send("stream.can=1#")}>开启 CAN 返回帧透传</button>
          </article>

          <article className="panel terminalPanel">
            <div className="panelTitle"><div><span>通信终端</span><small>USART2 / ZIGBEE</small></div><button className="textButton" onClick={() => setLogs([])}>清空</button></div>
            <div className="terminal" aria-live="polite">{logs.length === 0 ? <p className="empty">暂无通信记录</p> : logs.map((log, index) => <p key={`${log.time}-${index}`} className={log.direction.toLowerCase()}><time>{log.time}</time><b>{log.direction}</b><span>{log.text}</span></p>)}</div>
            <form onSubmit={(e) => { e.preventDefault(); void send(rawCommand); }}><input aria-label="原始命令" value={rawCommand} onChange={(e) => setRawCommand(e.target.value)} placeholder="motor.speed=1,300,20#" /><button className="primary">发送</button></form>
          </article>
        </aside>
      </section>
      <footer><span>协议 v1 · 115200 / 8N1 · 命令以 # 结束</span><span>Chrome / Edge Web Serial</span></footer>
    </main>
  );
}
