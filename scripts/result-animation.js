// Recorded MATLAB geometry and trajectories. No solver runs in the browser.
(() => {
  const ns='http://www.w3.org/2000/svg';
  const svgNode=(tag,attrs={})=>{const n=document.createElementNS(ns,tag); for(const [k,v] of Object.entries(attrs)) n.setAttribute(k,v); return n;};
  const reduced=matchMedia('(prefers-reduced-motion: reduce)');
  document.querySelectorAll('.result-animation').forEach(host=>{
    const poster=host.querySelector('img');
    let data, frame, playing=false, visible=false, progress=0, origin=0, start=0, generation=0, ready=false;
    const panel=document.createElement('div'); panel.className='animation-panel';
    const controls=document.createElement('div'); controls.className='animation-controls';
    const play=document.createElement('button'); play.type='button'; play.className='btn btn-primary'; play.textContent='Play';
    const reset=document.createElement('button'); reset.type='button'; reset.className='btn btn-outline-primary'; reset.textContent='Reset';
    const seek=document.createElement('input'); seek.type='range'; seek.min=0; seek.max=1000; seek.step=1; seek.value=0; seek.setAttribute('aria-label','Animation position');
    const speed=document.createElement('select'); speed.setAttribute('aria-label','Playback speed');
    for(const rate of [.5,1,2]){const option=document.createElement('option'); option.value=rate;option.textContent=rate+'×';option.selected=rate===1;speed.append(option);}
    const output=document.createElement('output'); output.className='animation-time';
    output.setAttribute('aria-live','off');
    const loop=document.createElement('input');loop.type='checkbox';loop.checked=true;loop.setAttribute('aria-label','Loop animation');
    const loopLabel=document.createElement('label');loopLabel.className='animation-loop';loopLabel.append(loop,document.createTextNode('Loop'));
    controls.append(play,reset,seek,speed,loopLabel,output);
    const note=document.createElement('p'); note.className='animation-note';
    let draw=()=>{};
    const pause=()=>{playing=false;cancelAnimationFrame(frame);play.textContent='Play';play.setAttribute('aria-pressed','false');};
    const paint=p=>{progress=Math.max(0,Math.min(1,p));seek.value=Math.round(progress*1000);draw(progress);};
    const tick=now=>{if(!playing)return;paint(origin+(now-start)/10000*Number(speed.value));if(progress>=1){if(loop.checked){paint(0);origin=0;start=now;frame=requestAnimationFrame(tick);}else pause();}else frame=requestAnimationFrame(tick);};
    const resume=()=>{if(!ready)return;if(progress>=1)paint(0);playing=true;origin=progress;start=performance.now();play.textContent='Pause';play.setAttribute('aria-pressed','true');frame=requestAnimationFrame(tick);};
    play.addEventListener('click',()=>playing?pause():resume());
    reset.addEventListener('click',()=>{pause();paint(0);});
    seek.addEventListener('input',()=>{pause();paint(Number(seek.value)/1000);});
    speed.addEventListener('change',()=>{if(playing){origin=progress;start=performance.now();}});
    document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});
    reduced.addEventListener('change',()=>{if(reduced.matches)pause();});
    async function load(url){
      const own=++generation;pause();ready=false;host.dataset.state='loading';
      host.setAttribute('aria-busy','true');
      host.querySelector('.animation-current-links')?.remove();
      play.disabled=true;seek.disabled=true;reset.disabled=true;
      try{
        const response=await fetch(url);if(!response.ok)throw Error('data');
        const value=await response.json();if(own!==generation)return;
        if(!value.title||!Array.isArray(value.times)||value.times.length<2)throw Error('schema');
        data=value;panel.replaceChildren();panel.hidden=false;controls.hidden=false;
        host.setAttribute('role','group');host.setAttribute('aria-label',data.title+' animation');
        host.querySelector('.animation-caption')?.remove();
        const heading=document.createElement('p');heading.className='animation-caption';heading.textContent=data.title;
        if(data.frames){
          const image=document.createElement('img');image.alt=data.title+' — recorded snapshot';panel.append(image);
          draw=p=>{const i=Math.min(data.frames.length-1,Math.floor(p*data.frames.length));image.src=new URL(data.frames[i],new URL(url,location.href)).href;output.textContent=`t = ${data.times[i].toFixed(2)} s · snapshot ${i+1}/${data.frames.length}`;};
        }else{
          const svg=svgNode('svg',{viewBox:'0 0 760 450',role:'img','aria-label':data.title});
          const [xmin,xmax,ymin,ymax]=data.bounds;
          const X=x=>110+(x-xmin)/(xmax-xmin)*624,Y=y=>386-(y-ymin)/(ymax-ymin)*352;
          const points=pts=>pts.map(p=>`${X(p[0])},${Y(p[1])}`).join(' ');
          svg.append(svgNode('rect',{x:110,y:34,width:624,height:352,fill:'#fff',stroke:'#9baec2'}));
          const number=n=>Math.abs(n)<1e-10?'0':Number(n.toPrecision(2)).toString();
          for(let i=0;i<=4;i++){
            const x=xmin+(xmax-xmin)*i/4,y=ymin+(ymax-ymin)*i/4;
            svg.append(svgNode('line',{x1:X(x),x2:X(x),y1:34,y2:386,stroke:'#e8edf3'}),svgNode('line',{x1:110,x2:734,y1:Y(y),y2:Y(y),stroke:'#e8edf3'}));
            for(const [text,attrs] of [[number(x),{x:X(x),y:413,'text-anchor':'middle'}],[number(y),{x:96,y:Y(y)+6,'text-anchor':'end'}]]){const label=svgNode('text',attrs);label.textContent=text;svg.append(label);}
          }
          const xlabel=svgNode('text',{x:405,y:442,'text-anchor':'middle'});xlabel.textContent=data.xLabel;
          const ylabel=svgNode('text',{x:24,y:212,transform:'rotate(-90 24 212)','text-anchor':'middle'});ylabel.textContent=data.yLabel;svg.append(xlabel,ylabel);
          for(const shape of data.staticSets||[])svg.append(svgNode('polygon',{points:points(shape.points),fill:shape.color||'#91c58b','fill-opacity':'.2',stroke:shape.color||'#43814b','stroke-width':1.2}));
          const envelopes=(data.sets||[]).map(shape=>{const n=svgNode('polygon',{points:points(shape.points),fill:shape.color||'#397cce','fill-opacity':'.025',stroke:shape.color||'#397cce','stroke-opacity':'.12','stroke-width':.6});svg.append(n);return n;});
          const traces=(data.series||[]).map((series,index)=>{const line=svgNode('polyline',{fill:'none',stroke:series.color||'#c64b29','stroke-width':index?1.2:2,'stroke-opacity':index?.45:1,'vector-effect':'non-scaling-stroke'});const dot=svgNode('circle',{r:index?2:4,fill:series.color||'#c64b29'});svg.append(line,dot);return {line,dot,series};});
          panel.append(svg);
          draw=p=>{
            const sequenceIndex=Math.min(data.times.length-1,Math.floor(p*data.times.length));
            const t=data.timeKind==='sequence'?data.times[sequenceIndex]:data.times[0]+p*(data.times.at(-1)-data.times[0]);
            let active=-1;for(let i=0;i<(data.sets||[]).length;i++)if(data.sets[i].time<=t+1e-9)active=i;
            envelopes.forEach((n,i)=>{n.setAttribute('fill-opacity',i===active?'.25':'.025');n.setAttribute('stroke-opacity',i===active?'1':'.12');n.setAttribute('stroke-width',i===active?'1.8':'.6');});
            for(const {line,dot,series} of traces){const pts=series.samples.filter(q=>q[0]<=t+1e-9);line.setAttribute('points',points(pts.map(q=>q.slice(1))));dot.setAttribute('visibility',pts.length?'visible':'hidden');if(pts.length){dot.setAttribute('cx',X(pts.at(-1)[1]));dot.setAttribute('cy',Y(pts.at(-1)[2]));}}
            output.textContent=data.timeKind==='sequence'?(data.stepLabels?.[sequenceIndex]||`Set ${sequenceIndex+1}/${data.times.length}`):`t = ${t.toFixed(2)} s`;
            seek.setAttribute('aria-valuetext',output.textContent);
          };
        }
        note.textContent=data.note;
        host.querySelector('.animation-diagnostic')?.remove();
        let diagnostic;
        if(data.diagnostics?.length){diagnostic=document.createElement('p');diagnostic.className='animation-diagnostic';diagnostic.textContent=data.diagnostics.join(' ');}
        host.querySelector('.animation-data-notes')?.remove();
        const details=document.createElement('details');details.className='animation-data-notes';
        const summary=document.createElement('summary');summary.textContent='About these data';details.append(summary);
        const description=document.createElement('p');description.textContent=[data.geometryNote,...(data.alignmentNotes||[])].filter(Boolean).join(' ');details.append(description);
        let legend=host.querySelector('.animation-legend');if(legend)legend.remove();legend=document.createElement('div');legend.className='animation-legend';
        for(const item of data.legend||[]){const label=document.createElement('span');const swatch=document.createElement('i');swatch.style.background=item.color;label.append(swatch,document.createTextNode(item.label));legend.append(label);}
        host.append(heading,controls,panel,legend,note,details);if(poster)poster.hidden=true;host.dataset.state='ready';host.setAttribute('aria-busy','false');ready=true;play.disabled=false;seek.disabled=false;reset.disabled=false;paint(0);
        if(diagnostic)host.append(diagnostic);
        host.querySelector('.animation-current-links')?.remove();
        if(host.dataset.showLinks==='true'){
          const links=document.createElement('p');links.className='animation-links animation-current-links';const base=new URL(url,location.href);
          for(const [title,href] of [['Run record & checks','../../pages/reproduction/index.html#'+data.caseId],['MATLAB source','../results/full-suite/'+data.caseId+'/source.m'],['Animated SVG',base.href.replace(/\.json$/,'.svg')],['Animation data',base.href]]){const a=document.createElement('a');a.textContent=title;a.href=new URL(href,base).href;if(title==='Animated SVG'||title==='Animation data')a.download='';links.append(a);}
          host.append(links);
        }
        host.dispatchEvent(new CustomEvent('animationready',{detail:{caseId:data.caseId}}));
        if(host.dataset.autoplay==='true'&&visible&&!reduced.matches)resume();
      }catch(error){if(own!==generation)return;host.dataset.state='error';host.setAttribute('aria-busy','false');panel.hidden=true;controls.hidden=true;host.querySelector('.animation-caption')?.remove();host.querySelector('.animation-legend')?.remove();host.querySelector('.animation-data-notes')?.remove();host.querySelector('.animation-diagnostic')?.remove();if(poster)poster.hidden=false;note.textContent='Animation unavailable. The static MATLAB result and source data remain available.';host.append(note);}
    }
    const select=host.closest('section')?.querySelector('[data-animation-select]');
    if(select)select.addEventListener('change',()=>{host.dataset.source=select.value;const option=select.options[select.selectedIndex];if(poster&&option.dataset.poster){poster.src=option.dataset.poster;poster.alt=option.textContent+' — static MATLAB result';}load(select.value);});
    const observer=new IntersectionObserver(entries=>{visible=entries[0].isIntersecting;if(visible&&!data&&!ready)load(host.dataset.source);if(!visible)pause();},{threshold:.15});
    observer.observe(host);
  });
})();
