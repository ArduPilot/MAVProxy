import{c as o,M as a,Y as i,u as r,l as m,m as c,n as e,t as n,q as l}from"./index-BUkzlHeJ.js";const d={class:"flex items-center w-fit min-w-[8rem] max-w-[9rem] h-12 p-1 text-white justify-center"},f={class:"flex flex-col items-start justify-center ml-1 min-w-[4rem] max-w-[6rem] select-none"},p={class:"font-mono font-semibold leading-4 text-end w-fit"},x={class:"font-mono text-sm font-semibold leading-4 text-end w-fit"},w=o({__name:"SatelliteIndicator",setup(_){a.registerUsage(i.gpsFixType),a.registerUsage(i.gpsVisibleSatellites);const t=r();return(g,s)=>(m(),c("div",d,[s[0]||(s[0]=e("span",{class:"relative w-[2.25rem] mdi mdi-satellite-variant text-4xl"},null,-1)),e("div",f,[e("span",p,n(l(t).statusGPS.visibleSatellites)+" sats",1),e("span",x,n(l(t).statusGPS.fixType),1)])]))}});export{w as default};
