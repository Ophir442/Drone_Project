import React, { useRef, useEffect, useCallback } from 'react';
import { simToCanvas, canvasToSim } from '../utils/geometry';
import Legend from './Legend';

const DRONE_COLORS = ['#6366f1', '#ec4899', '#f59e0b', '#22c55e', '#06b6d4', '#f97316', '#8b5cf6'];

function drawArrow(ctx, fromX, fromY, toX, toY, color, lineWidth = 1.5) {
  const dx = toX - fromX;
  const dy = toY - fromY;
  const len = Math.sqrt(dx * dx + dy * dy);
  if (len < 10) return;

  ctx.strokeStyle = color;
  ctx.lineWidth = lineWidth;
  ctx.setLineDash([6, 4]);
  ctx.beginPath();
  ctx.moveTo(fromX, fromY);
  ctx.lineTo(toX, toY);
  ctx.stroke();
  ctx.setLineDash([]);

  const midX = (fromX + toX) / 2;
  const midY = (fromY + toY) / 2;
  const angle = Math.atan2(dy, dx);
  const headLen = 5;

  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(midX + headLen * Math.cos(angle), midY + headLen * Math.sin(angle));
  ctx.lineTo(midX - headLen * Math.cos(angle - Math.PI / 5), midY - headLen * Math.sin(angle - Math.PI / 5));
  ctx.lineTo(midX - headLen * Math.cos(angle + Math.PI / 5), midY - headLen * Math.sin(angle + Math.PI / 5));
  ctx.closePath();
  ctx.fill();
}

export default function SimulationMap({ state, gridWidth, gridHeight, selected, onSelect, onMapClick }) {
  const canvasRef = useRef(null);
  const padding = 45;

  const draw = useCallback((ctx, width, height) => {
    if (!state) return;

    const toCanvas = (sx, sy) => simToCanvas(sx, sy, width, height, gridWidth, gridHeight, padding);

    ctx.fillStyle = '#0c0c16';
    ctx.fillRect(0, 0, width, height);

    // Grid
    ctx.strokeStyle = '#14142a';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 10; i++) {
      const { x: gx } = toCanvas((gridWidth / 10) * i, 0);
      const { y: gy } = toCanvas(0, (gridHeight / 10) * i);
      ctx.beginPath();
      ctx.moveTo(gx, padding);
      ctx.lineTo(gx, height - padding);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(padding, gy);
      ctx.lineTo(width - padding, gy);
      ctx.stroke();
    }

    // Axis labels
    ctx.fillStyle = '#334155';
    ctx.font = '9px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    for (let i = 0; i <= 10; i += 2) {
      const val = (gridWidth / 10) * i;
      const { x: lx } = toCanvas(val, 0);
      ctx.fillText(val.toFixed(0), lx, height - padding + 14);
      const { y: ly } = toCanvas(0, val);
      ctx.textAlign = 'right';
      ctx.fillText(val.toFixed(0), padding - 8, ly);
      ctx.textAlign = 'center';
    }
    ctx.textBaseline = 'alphabetic';

    // Routes
    state.drones.forEach((drone, di) => {
      if (drone.is_idle || drone.planned_route.length === 0) return;
      const color = DRONE_COLORS[di % DRONE_COLORS.length];
      const isSel = selected?.type === 'drone' && selected?.data?.id === drone.id;
      const alpha = isSel ? 'cc' : '55';

      let prevPt = toCanvas(drone.pos.x, drone.pos.y);
      for (let i = drone.route_progress; i < drone.planned_route.length; i++) {
        const node = drone.planned_route[i];
        const pt = toCanvas(node.pos.x, node.pos.y);
        drawArrow(ctx, prevPt.x, prevPt.y, pt.x, pt.y, color + alpha, isSel ? 2 : 1);
        prevPt = pt;
      }
    });

    // Base
    const basePt = toCanvas(state.base.x, state.base.y);
    ctx.fillStyle = '#e2e8f0';
    ctx.strokeStyle = '#94a3b8';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.rect(basePt.x - 8, basePt.y - 3, 16, 11);
    ctx.fill();
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(basePt.x - 10, basePt.y - 3);
    ctx.lineTo(basePt.x, basePt.y - 12);
    ctx.lineTo(basePt.x + 10, basePt.y - 3);
    ctx.closePath();
    ctx.fillStyle = '#6366f1';
    ctx.fill();
    ctx.stroke();
    ctx.fillStyle = '#1e1e32';
    ctx.fillRect(basePt.x - 2, basePt.y + 1, 4, 7);

    ctx.fillStyle = '#64748b';
    ctx.font = '9px Inter, sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('BASE', basePt.x, basePt.y + 20);

    // Bakeries
    state.bakeries.forEach(bakery => {
      const pt = toCanvas(bakery.pos.x, bakery.pos.y);
      const isSel = selected?.type === 'bakery' && selected?.data?.id === bakery.id;

      if (isSel) {
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 20, 0, Math.PI * 2);
        ctx.stroke();
      }

      const s = 10;
      ctx.fillStyle = '#f59e0b';
      ctx.beginPath();
      ctx.roundRect(pt.x - s, pt.y - s + 4, s * 2, s * 2 - 4, 3);
      ctx.fill();
      ctx.fillStyle = '#d97706';
      ctx.beginPath();
      ctx.moveTo(pt.x - s - 2, pt.y - s + 4);
      ctx.lineTo(pt.x + s + 2, pt.y - s + 4);
      ctx.lineTo(pt.x + s, pt.y - s + 8);
      ctx.lineTo(pt.x - s, pt.y - s + 8);
      ctx.closePath();
      ctx.fill();
      ctx.fillStyle = '#fef3c7';
      ctx.fillRect(pt.x - 3, pt.y - 1, 6, 5);

      // Inventory bar
      const barW = 24;
      const barH = 4;
      const barY = pt.y + s + 4;
      ctx.fillStyle = '#1e1e32';
      ctx.beginPath();
      ctx.roundRect(pt.x - barW / 2, barY, barW, barH, 2);
      ctx.fill();

      const ratio = bakery.current_inventory / bakery.capacity;
      if (ratio > 0) {
        ctx.fillStyle = ratio > 0.5 ? '#22c55e' : ratio > 0.2 ? '#f59e0b' : '#ef4444';
        ctx.beginPath();
        ctx.roundRect(pt.x - barW / 2, barY, Math.max(2, barW * ratio), barH, 2);
        ctx.fill();
      }

      ctx.fillStyle = '#94a3b8';
      ctx.font = 'bold 9px Inter, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(`B${bakery.id}`, pt.x, barY + barH + 11);
    });

    // Customers
    state.customers.forEach(customer => {
      const pt = toCanvas(customer.pos.x, customer.pos.y);
      const isSel = selected?.type === 'customer' && selected?.data?.id === customer.id;
      const priorityNorm = Math.min(customer.priority_weight / 10, 1);

      if (isSel) {
        ctx.strokeStyle = '#6366f1';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 14, 0, Math.PI * 2);
        ctx.stroke();
      }

      const bodyColor = priorityNorm > 0.6 ? '#ef4444' : priorityNorm > 0.3 ? '#f59e0b' : '#3b82f6';
      ctx.fillStyle = bodyColor;
      ctx.beginPath();
      ctx.arc(pt.x, pt.y - 4, 3.5, 0, Math.PI * 2);
      ctx.fill();
      ctx.beginPath();
      ctx.moveTo(pt.x - 4, pt.y + 7);
      ctx.lineTo(pt.x, pt.y);
      ctx.lineTo(pt.x + 4, pt.y + 7);
      ctx.closePath();
      ctx.fill();

      // Priority badge
      const badgeX = pt.x + 7;
      const badgeY = pt.y - 8;
      const pw = customer.priority_weight;
      ctx.fillStyle = bodyColor;
      ctx.beginPath();
      ctx.arc(badgeX, badgeY, 6, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = '#fff';
      ctx.font = 'bold 7px JetBrains Mono, monospace';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(pw >= 10 ? '9+' : pw.toFixed(0), badgeX, badgeY);
      ctx.textBaseline = 'alphabetic';

      ctx.fillStyle = '#64748b';
      ctx.font = '9px Inter, sans-serif';
      ctx.textAlign = 'center';
      ctx.fillText(`C${customer.id}`, pt.x, pt.y + 16);
    });

    // Drones
    state.drones.forEach((drone, di) => {
      const pt = toCanvas(drone.pos.x, drone.pos.y);
      const color = DRONE_COLORS[di % DRONE_COLORS.length];
      const isSel = selected?.type === 'drone' && selected?.data?.id === drone.id;

      if (isSel) {
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(pt.x, pt.y, 12, 0, Math.PI * 2);
        ctx.stroke();
      }

      const bodyColor = drone.is_idle ? '#475569' : color;
      ctx.fillStyle = bodyColor;
      ctx.beginPath();
      ctx.arc(pt.x, pt.y, 4, 0, Math.PI * 2);
      ctx.fill();

      ctx.strokeStyle = bodyColor;
      ctx.lineWidth = 1.5;
      const armLen = 7;
      for (let a = 0; a < 4; a++) {
        const angle = (a * Math.PI / 2) + Math.PI / 4;
        const ax = pt.x + Math.cos(angle) * armLen;
        const ay = pt.y + Math.sin(angle) * armLen;
        ctx.beginPath();
        ctx.moveTo(pt.x, pt.y);
        ctx.lineTo(ax, ay);
        ctx.stroke();
        ctx.fillStyle = bodyColor;
        ctx.beginPath();
        ctx.arc(ax, ay, 2.5, 0, Math.PI * 2);
        ctx.fill();
      }

      ctx.fillStyle = '#e2e8f0';
      ctx.font = 'bold 9px JetBrains Mono, monospace';
      ctx.textAlign = 'left';
      ctx.fillText(`D${drone.id}`, pt.x + 11, pt.y - 2);

      if (drone.current_load > 0) {
        ctx.fillStyle = '#f59e0b';
        ctx.font = '8px JetBrains Mono, monospace';
        ctx.fillText(`${drone.current_load}`, pt.x + 11, pt.y + 8);
      }
    });

  }, [state, gridWidth, gridHeight, padding, selected]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const parent = canvas.parentElement;
    const dpr = window.devicePixelRatio || 1;
    const w = parent.clientWidth;
    const h = parent.clientHeight;
    canvas.width = w * dpr;
    canvas.height = h * dpr;
    canvas.style.width = w + 'px';
    canvas.style.height = h + 'px';
    const ctx = canvas.getContext('2d');
    ctx.scale(dpr, dpr);
    draw(ctx, w, h);
  }, [draw]);

  useEffect(() => {
    const handleResize = () => {
      const canvas = canvasRef.current;
      if (!canvas) return;
      const parent = canvas.parentElement;
      const dpr = window.devicePixelRatio || 1;
      const w = parent.clientWidth;
      const h = parent.clientHeight;
      canvas.width = w * dpr;
      canvas.height = h * dpr;
      canvas.style.width = w + 'px';
      canvas.style.height = h + 'px';
      const ctx = canvas.getContext('2d');
      ctx.scale(dpr, dpr);
      draw(ctx, w, h);
    };
    window.addEventListener('resize', handleResize);
    return () => window.removeEventListener('resize', handleResize);
  }, [draw]);

  const handleClick = useCallback((e) => {
    if (!state || !onSelect) return;
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    const w = rect.width;
    const h = rect.height;

    const toCanvas = (sx, sy) => simToCanvas(sx, sy, w, h, gridWidth, gridHeight, padding);

    for (const drone of state.drones) {
      const pt = toCanvas(drone.pos.x, drone.pos.y);
      if (Math.hypot(cx - pt.x, cy - pt.y) < 14) {
        onSelect({ type: 'drone', data: drone });
        return;
      }
    }
    for (const customer of state.customers) {
      const pt = toCanvas(customer.pos.x, customer.pos.y);
      if (Math.hypot(cx - pt.x, cy - pt.y) < 14) {
        onSelect({ type: 'customer', data: customer });
        return;
      }
    }
    for (const bakery of state.bakeries) {
      const pt = toCanvas(bakery.pos.x, bakery.pos.y);
      if (Math.hypot(cx - pt.x, cy - pt.y) < 14) {
        onSelect({ type: 'bakery', data: bakery });
        return;
      }
    }
    onSelect(null);
  }, [state, gridWidth, gridHeight, onSelect, padding]);

  const handleDoubleClick = useCallback((e) => {
    if (!onMapClick) return;
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const cx = e.clientX - rect.left;
    const cy = e.clientY - rect.top;
    const simPos = canvasToSim(cx, cy, rect.width, rect.height, gridWidth, gridHeight, padding);
    onMapClick(simPos);
  }, [onMapClick, gridWidth, gridHeight, padding]);

  return (
    <div style={{ position: 'relative', width: '100%', height: '100%' }}>
      <canvas
        ref={canvasRef}
        onClick={handleClick}
        onDoubleClick={handleDoubleClick}
        style={{
          display: 'block',
          width: '100%',
          height: '100%',
          cursor: 'crosshair',
          background: '#0c0c16',
        }}
      />
      <Legend />
    </div>
  );
}
