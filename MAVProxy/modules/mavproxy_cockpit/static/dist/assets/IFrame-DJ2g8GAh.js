import{c as B,b9 as O,K as T,J as $,I,r,L as N,ba as b,au as W,k as h,w as z,l as x,m as j,n as c,H as A,D as y,aN as D,q as n,O as S,al as K,G as s,N as u,V as G,P as H,Q as F,R as k,U as P,bb as q,a5 as M,T as E,aB as J,bc as Q,W as X,bd as Y,F as Z,be as ee,_ as ae}from"./index-BUkzlHeJ.js";const te={class:"w-full h-full"},se=["src"],le={class:"flex items-center justify-between"},oe=B({__name:"IFrame",props:{widget:{}},setup(U){O(e=>({"6f56b4ce":R.value}));const _=T(),i=$(),a=I(U).widget,m=r(!1),p=r(0),l=r(a.value.options.source),d=r(!1),f=r(""),v=e=>ee(e)?!0:"URL is not valid.",g=()=>{const e=v(l.value);if(e!==!0){f.value=`${e} Please enter a valid URL.`,d.value=!0;return}a.value.options.source=l.value,f.value=`IFrame URL sucessfully updated to '${l.value}'.`,d.value=!0};N(()=>{Object.keys(a.value.options).length===0&&(a.value.options={source:b},l.value=b)});const{width:w,height:V}=W(),C=h(()=>{let e="";return e=e.concat(" ","position: absolute;"),e=e.concat(" ",`left: ${a.value.position.x*w.value}px;`),e=e.concat(" ",`top: ${a.value.position.y*V.value}px;`),e=e.concat(" ",`width: ${a.value.size.width*w.value}px;`),e=e.concat(" ",`height: ${a.value.size.height*V.value}px;`),i.editingMode&&(e=e.concat(" ","pointer-events:none; border:0;")),i.isWidgetVisible(a.value)||(e=e.concat(" ","display: none;")),e}),R=h(()=>(100-p.value)/100);function L(){console.log("Finished loading"),m.value=!0}return z(a,()=>{i.widgetManagerVars(a.value.hash).configMenuOpen===!1&&v(l.value)!==!0&&(l.value=a.value.options.source)},{deep:!0}),(e,t)=>(x(),j(Z,null,[c("div",te,[(x(),A(K,{to:".widgets-view"},[y(c("iframe",{src:n(a).options.source,style:S(C.value),frameborder:"0",onLoad:L},null,44,se),[[D,m.value]])])),s(X,{modelValue:n(i).widgetManagerVars(n(a).hash).configMenuOpen,"onUpdate:modelValue":t[3]||(t[3]=o=>n(i).widgetManagerVars(n(a).hash).configMenuOpen=o),"min-width":"400","max-width":"35%"},{default:u(()=>[s(G,{class:"pa-2",style:S(n(_).globalGlassMenuStyles)},{default:u(()=>[s(H,{class:"text-center"},{default:u(()=>t[5]||(t[5]=[F("Settings")])),_:1}),s(k,null,{default:u(()=>[t[6]||(t[6]=c("p",null,"Iframe Source",-1)),c("div",le,[s(P,{modelValue:l.value,"onUpdate:modelValue":t[0]||(t[0]=o=>l.value=o),variant:"filled",outlined:"",rules:[v],onKeydown:q(g,["enter"])},null,8,["modelValue","rules"]),y(s(M,{icon:"mdi-check",class:"mx-1 mb-5 bg-[#FFFFFF22]",rounded:"lg",flat:"",onClick:g},null,512),[[E,"Set",void 0,{bottom:!0}]])])]),_:1}),s(k,null,{default:u(()=>[s(J,{modelValue:p.value,"onUpdate:modelValue":t[1]||(t[1]=o=>p.value=o),label:"Transparency",color:"white",min:0,max:90},null,8,["modelValue"])]),_:1}),s(Q,{class:"flex justify-end"},{default:u(()=>[s(M,{color:"white",onClick:t[2]||(t[2]=o=>n(i).widgetManagerVars(n(a).hash).configMenuOpen=!1)},{default:u(()=>t[7]||(t[7]=[F(" Close ")])),_:1})]),_:1})]),_:1},8,["style"])]),_:1},8,["modelValue"])]),s(Y,{"open-snackbar":d.value,message:f.value,duration:3e3,"close-button":!1,"onUpdate:openSnackbar":t[4]||(t[4]=o=>d.value=o)},null,8,["open-snackbar","message"])],64))}}),ie=ae(oe,[["__scopeId","data-v-2823bce5"]]);export{ie as default};
