const pages=new Set(['system','chassis','map','qr','vision','params','arm']);
const parameterTabs=new Set(['material','turntable','vision','motion','radar']);
const mapTabs=new Set(['monitor','radar','offline']);
export function parsePageRoute(hash){
  const [page,tab,key]=String(hash??'').replace(/^#/,'').split('/');
  return {page:pages.has(page)?page:'system',tab:page==='params'?(parameterTabs.has(tab)?tab:'material'):page==='map'?(mapTabs.has(tab)?tab:'monitor'):tab,key};
}
/** In-memory context only; the caller performs navigation without device I/O. */
export function createNavigationMemory(){
  const scroll=new Map();let source=null;
  return {save(hash,top){scroll.set(hash,top);},restore:hash=>scroll.get(hash)??0,
    enterParameters(origin){source=origin??null;return source;},get source(){return source;}};
}
