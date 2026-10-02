export function parameterSurface(){
  class Element{
    constructor(tag){this.tag=tag;this.children=[];this.dataset={};this.attributes={};this.textContent='';this.value='';this.disabled=false;this.classList={toggle(){}};}
    append(...children){this.children.push(...children);}
    setAttribute(key,value){this.attributes[key]=String(value);}
    querySelector(selector){return walk(this).find(item=>selector.startsWith('#')?item.id===selector.slice(1):false);}
    get lastChild(){return this.children.at(-1);}
  }
  const previous=globalThis.document;
  globalThis.document={createElement:tag=>new Element(tag)};
  const root=new Element('div');
  return {root,all:()=>walk(root),input:(kind,key)=>walk(root).find(e=>e.id===`parameter-${kind}-${key}`),restore(){globalThis.document=previous;}};
}
function walk(root){return [root,...root.children.flatMap(walk)];}
