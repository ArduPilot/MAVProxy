import{c as l,u as o,l as r,m as i,n as d,v as c,q as a,X as n,y as u,z as m,E as g}from"./index-BUkzlHeJ.js";const f=["disabled"],v=l({__name:"ChangeAltitudeCommander",setup(C){const t=o(),s=()=>{n.value=!0,u(()=>{n.value=!1,t.changeAlt()},{command:"Altitude Change"},m(g.ALT_CHANGE))};return(h,e)=>(r(),i("button",{class:c(["relative flex items-center justify-center w-32 p-1 rounded-md shadow-inner h-9",a(t).flying?"bg-slate-800/60 hover:bg-slate-400/60":"bg-slate-400/60 cursor-not-allowed"]),disabled:!a(t).flying,onClick:e[0]||(e[0]=p=>s())},e[1]||(e[1]=[d("span",{class:"inline-block font-extrabold align-middle text-white"}," Change Alt ",-1)]),10,f))}});export{v as default};
