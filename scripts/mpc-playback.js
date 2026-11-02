// Playback of recorded MATLAB output; no simulation is performed in-browser.
document.querySelectorAll('.mpc-playback').forEach(async host => {
  try {
    const response = await fetch(host.dataset.source);
    if (!response.ok) throw new Error('Playback data unavailable');
    const data = await response.json();
    if (!Array.isArray(data.x) || data.x.length < 2 || data.x.length !== data.t.length) throw new Error('Invalid trajectory');
    const ns = 'http://www.w3.org/2000/svg';
    const make = (tag, attrs = {}) => {
      const el = document.createElementNS(ns, tag);
      Object.entries(attrs).forEach(([key,value]) => el.setAttribute(key,value));
      return el;
    };
    const svg = make('svg', { viewBox:'0 0 800 520', role:'img', 'aria-label':'Recorded linear MPC simulation with reachable sets and terminal region' });
    const X = x => 65 + (x + 2) / 5.3 * 710;
    const Y = y => 460 - (y + 3.2) / 4.5 * 420;
    const points = array => array.map(p => `${X(p[0])},${Y(p[1])}`).join(' ');
    svg.append(make('rect',{x:65,y:40,width:710,height:420,fill:'white',stroke:'#aab8c8'}));
    for (let x=-2;x<=3;x++) {
      svg.append(make('line',{x1:X(x),x2:X(x),y1:40,y2:460,stroke:'#e3e9ef'}));
      const t=make('text',{x:X(x),y:485,'text-anchor':'middle',fill:'#23364b'}); t.textContent=x; svg.append(t);
    }
    for (let y=-3;y<=1;y++) {
      svg.append(make('line',{x1:65,x2:775,y1:Y(y),y2:Y(y),stroke:'#e3e9ef'}));
      const t=make('text',{x:50,y:Y(y)+5,'text-anchor':'end',fill:'#23364b'}); t.textContent=y; svg.append(t);
    }
    svg.append(make('polygon',{points:points(data.terminal),fill:'#b8dcb2',stroke:'#47804c'}));
    data.sets.forEach(set => svg.append(make('polygon',{points:points(set),fill:'#c2d8f2','fill-opacity':'.4',stroke:'#185caa','stroke-width':'.8'})));
    const trace=make('polyline',{fill:'none',stroke:'#c83d24','stroke-width':2});
    const dot=make('circle',{r:5,fill:'#c83d24'}); svg.append(trace,dot);
    const label=make('text',{x:420,y:515,'text-anchor':'middle',fill:'#23364b'}); label.textContent='Position x₁'; svg.append(label);
    const vlabel=make('text',{x:15,y:250,transform:'rotate(-90 15 250)','text-anchor':'middle',fill:'#23364b'}); vlabel.textContent='Velocity x₂'; svg.append(vlabel);
    const controls=document.createElement('div'); controls.className='playback-controls';
    const button=document.createElement('button'); button.className='btn btn-primary'; button.textContent='Play'; button.type='button';
    const reset=document.createElement('button'); reset.className='btn btn-outline-primary'; reset.textContent='Reset'; reset.type='button';
    const slider=document.createElement('input'); slider.type='range'; slider.min=0; slider.max=data.x.length-1; slider.value=0; slider.setAttribute('aria-label','Recorded simulation time');
    const status=document.createElement('output'); status.setAttribute('aria-label','Simulation time');
    controls.append(button,reset,slider,status);
    const note=document.createElement('p'); note.className='small text-secondary'; note.textContent='Recorded MATLAB simulation. Blue: computed time-point reachable sets; red: simulated state; green: terminal polytope. Playback takes 8 seconds and is not a timing benchmark.';
    const fallback=host.querySelector('img'); if (fallback) fallback.hidden=true;
    host.append(svg,controls,note);
    let playing=false, frame, started=0, initial=0;
    const draw = i => {
      i=Math.max(0,Math.min(data.x.length-1,Math.round(i))); slider.value=i;
      trace.setAttribute('points',points(data.x.slice(0,i+1)));
      dot.setAttribute('cx',X(data.x[i][0])); dot.setAttribute('cy',Y(data.x[i][1]));
      status.textContent=`t = ${data.t[i].toFixed(2)} s`;
    };
    const pause=()=>{playing=false; cancelAnimationFrame(frame);button.textContent='Play';};
    const tick=now=>{
      if (!playing) return;
      const i=initial+(now-started)/8000*(data.x.length-1); draw(i);
      if(i>=data.x.length-1) pause(); else frame=requestAnimationFrame(tick);
    };
    button.addEventListener('click',()=>{
      if(playing) return pause();
      if(+slider.value>=data.x.length-1) draw(0);
      playing=true; initial=+slider.value; started=performance.now(); button.textContent='Pause'; frame=requestAnimationFrame(tick);
    });
    reset.addEventListener('click',()=>{pause();draw(0);});
    slider.addEventListener('input',()=>{pause();draw(+slider.value);});
    document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});
    draw(0);
  } catch {
    // Static MATLAB plot remains visible when enhancement is unavailable.
  }
});
