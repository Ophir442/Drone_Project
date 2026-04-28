import React, { useState } from 'react';

export default function AddCustomerModal({ initialPos, gridWidth, gridHeight, onAdd, onClose }) {
  const [x, setX] = useState(initialPos ? initialPos.x.toFixed(1) : '50');
  const [y, setY] = useState(initialPos ? initialPos.y.toFixed(1) : '50');
  const [qty, setQty] = useState('3');

  const handleSubmit = (e) => {
    e.preventDefault();
    const px = Math.max(0, Math.min(gridWidth, parseFloat(x) || 50));
    const py = Math.max(0, Math.min(gridHeight, parseFloat(y) || 50));
    const pq = Math.max(1, parseInt(qty) || 1);
    onAdd(px, py, pq, '');
  };

  return (
    <div style={styles.overlay} onClick={onClose}>
      <div style={styles.modal} onClick={e => e.stopPropagation()}>
        <div style={styles.header}>
          <span style={styles.title}>Add Customer</span>
          <button style={styles.closeBtn} onClick={onClose}>&times;</button>
        </div>

        <form onSubmit={handleSubmit} style={styles.form}>
          <div style={styles.rowDouble}>
            <div style={styles.halfField}>
              <label style={styles.label}>X</label>
              <input
                style={styles.input}
                type="number"
                min="0"
                max={gridWidth}
                step="0.1"
                value={x}
                onChange={e => setX(e.target.value)}
                required
              />
            </div>
            <div style={styles.halfField}>
              <label style={styles.label}>Y</label>
              <input
                style={styles.input}
                type="number"
                min="0"
                max={gridHeight}
                step="0.1"
                value={y}
                onChange={e => setY(e.target.value)}
                required
              />
            </div>
          </div>

          <div style={styles.row}>
            <label style={styles.label}>Quantity</label>
            <input
              style={styles.input}
              type="number"
              min="1"
              max="20"
              value={qty}
              onChange={e => setQty(e.target.value)}
              required
            />
          </div>

          <div style={styles.actions}>
            <button type="button" style={styles.cancelBtn} onClick={onClose}>Cancel</button>
            <button type="submit" style={styles.submitBtn}>Add</button>
          </div>
        </form>
      </div>
    </div>
  );
}

const styles = {
  overlay: {
    position: 'fixed',
    inset: 0,
    background: 'rgba(0,0,0,0.6)',
    backdropFilter: 'blur(4px)',
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'center',
    zIndex: 1000,
  },
  modal: {
    background: '#14141f',
    borderRadius: 12,
    border: '1px solid #2d2d4a',
    width: 320,
    boxShadow: '0 24px 64px rgba(0,0,0,0.5)',
    overflow: 'hidden',
  },
  header: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: '14px 18px',
    borderBottom: '1px solid #1e1e32',
  },
  title: {
    fontSize: 14,
    fontWeight: 600,
    color: '#e2e8f0',
  },
  closeBtn: {
    background: 'none',
    border: 'none',
    color: '#64748b',
    fontSize: 20,
    cursor: 'pointer',
    padding: '0 4px',
    lineHeight: 1,
  },
  form: {
    padding: '14px 18px 18px',
    display: 'flex',
    flexDirection: 'column',
    gap: 12,
  },
  row: {
    display: 'flex',
    flexDirection: 'column',
    gap: 4,
  },
  rowDouble: {
    display: 'flex',
    gap: 10,
  },
  halfField: {
    flex: 1,
    display: 'flex',
    flexDirection: 'column',
    gap: 4,
  },
  label: {
    fontSize: 12,
    color: '#64748b',
    fontWeight: 500,
  },
  input: {
    padding: '8px 10px',
    background: '#0c0c16',
    border: '1px solid #2d2d4a',
    borderRadius: 6,
    color: '#e2e8f0',
    fontSize: 13,
    fontFamily: "'JetBrains Mono', monospace",
    outline: 'none',
    width: '100%',
  },
  actions: {
    display: 'flex',
    gap: 8,
    marginTop: 4,
  },
  cancelBtn: {
    flex: 1,
    padding: '9px',
    background: '#1e1e32',
    color: '#94a3b8',
    border: '1px solid #2d2d4a',
    borderRadius: 6,
    cursor: 'pointer',
    fontSize: 13,
    fontFamily: 'inherit',
    fontWeight: 500,
  },
  submitBtn: {
    flex: 1,
    padding: '9px',
    background: '#6366f1',
    color: '#fff',
    border: 'none',
    borderRadius: 6,
    cursor: 'pointer',
    fontSize: 13,
    fontFamily: 'inherit',
    fontWeight: 600,
  },
};
