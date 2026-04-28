import React from 'react';

export default function ControlPanel({ playing, speed, onPlayPause, onStep, onSpeedChange, onAddCustomer, onReset }) {
  return (
    <div style={styles.container}>
      <div style={styles.buttonRow}>
        <button
          style={{ ...styles.btn, ...(playing ? styles.btnActive : {}) }}
          onClick={onPlayPause}
        >
          {playing ? 'Pause' : 'Play'}
        </button>
        <button style={styles.btn} onClick={onStep} disabled={playing}>
          Step
        </button>
      </div>

      <div style={styles.sliderRow}>
        <span style={styles.sliderLabel}>Delay</span>
        <input
          type="range"
          min={100}
          max={2000}
          step={100}
          value={speed}
          onChange={(e) => onSpeedChange(parseInt(e.target.value))}
          style={styles.slider}
        />
        <span style={styles.sliderValue}>{speed}ms</span>
      </div>

      <div style={styles.actionRow}>
        <button style={styles.addBtn} onClick={onAddCustomer}>+ Customer</button>
        <button style={styles.resetBtn} onClick={onReset}>Reset</button>
      </div>

      <div style={styles.hint}>Double-click map to place a customer</div>
    </div>
  );
}

const styles = {
  container: {
    padding: '14px 20px',
    borderBottom: '1px solid #1e1e32',
  },
  buttonRow: {
    display: 'flex',
    gap: 8,
    marginBottom: 10,
  },
  btn: {
    flex: 1,
    padding: '8px 12px',
    background: '#1e1e32',
    color: '#e2e8f0',
    border: '1px solid #2d2d4a',
    borderRadius: 6,
    cursor: 'pointer',
    fontSize: 13,
    fontWeight: 500,
    fontFamily: 'inherit',
    transition: 'all 0.15s',
  },
  btnActive: {
    background: '#6366f1',
    borderColor: '#6366f1',
    color: '#fff',
  },
  sliderRow: {
    display: 'flex',
    alignItems: 'center',
    gap: 10,
    marginBottom: 10,
  },
  sliderLabel: {
    fontSize: 12,
    color: '#64748b',
    minWidth: 34,
  },
  slider: {
    flex: 1,
  },
  sliderValue: {
    fontSize: 11,
    color: '#94a3b8',
    minWidth: 48,
    textAlign: 'right',
    fontFamily: "'JetBrains Mono', monospace",
  },
  actionRow: {
    display: 'flex',
    gap: 8,
    marginBottom: 8,
  },
  addBtn: {
    flex: 1,
    padding: '7px 12px',
    background: 'transparent',
    color: '#22c55e',
    border: '1px dashed #22c55e40',
    borderRadius: 6,
    cursor: 'pointer',
    fontSize: 12,
    fontWeight: 500,
    fontFamily: 'inherit',
  },
  resetBtn: {
    padding: '7px 16px',
    background: 'transparent',
    color: '#ef4444',
    border: '1px solid #ef444440',
    borderRadius: 6,
    cursor: 'pointer',
    fontSize: 12,
    fontWeight: 500,
    fontFamily: 'inherit',
  },
  hint: {
    fontSize: 10,
    color: '#475569',
    textAlign: 'center',
  },
};
