import { frameCommand, LineDecoder } from './protocol.mjs?v=heading-hold-1';

// One open port, one reader and one explicit write at a time. No automatic commands.
export class SerialLink {
  port=null; reader=null; readTask=null; writeTask=null; closing=false; connecting=false;
  constructor({receive=()=>{},state=()=>{},error=()=>{}}={}) {
    this.receive=receive; this.state=state; this.error=error;
  }
  get connected(){return !!this.port && !this.closing;}
  async connect(serial,baudRate) {
    if(this.port || this.connecting) throw new Error('连接操作正在进行。');
    this.connecting=true; this.state('connecting');
    try {
      const port=await serial.requestPort();
      await port.open({baudRate,dataBits:8,stopBits:1,parity:'none',flowControl:'none'});
      this.port=port; this.closing=false; this.state('connected');
      this.readTask=this.read(port);
    } catch(error) { this.state('disconnected'); throw error; }
    finally {this.connecting=false;}
  }
  async read(port) {
    const decoder=new TextDecoder(); const lines=new LineDecoder();
    try {
      if(!port.readable) throw new Error('串口不可读取。');
      this.reader=port.readable.getReader();
      while(!this.closing) {
        const {value,done}=await this.reader.read();
        if(done) break;
        for(const line of lines.push(decoder.decode(value,{stream:true}))) this.receive(line);
      }
    } catch(error) {if(!this.closing) this.error(error);}
    finally {
      this.reader?.releaseLock(); this.reader=null;
      if(!this.closing) {
        this.closing=true; this.state('disconnected');
        await this.writeTask?.catch(()=>{});
        try{await port.close();}catch(error){this.error(error);}
        if(this.port===port) this.port=null;
        this.closing=false;
      }
    }
  }
  async send(command) {
    const frame=frameCommand(command);
    if(!this.connected || !this.port.writable) throw new Error('请先连接串口。');
    if(this.writeTask) throw new Error('上一条数据仍在发送，请稍后再试。');
    const writer=this.port.writable.getWriter();
    const task=writer.write(new TextEncoder().encode(frame));
    this.writeTask=task;
    try{await task;}finally{writer.releaseLock();this.writeTask=null;}
    return frame;
  }
  async disconnect() {
    const port=this.port;
    if(!port || this.closing) return;
    this.closing=true; this.state('disconnecting');
    try {
      await this.reader?.cancel();
      await this.readTask;
      await this.writeTask?.catch(()=>{});
      await port.close();
    } finally {this.port=null;this.closing=false;this.state('disconnected');}
  }
}
