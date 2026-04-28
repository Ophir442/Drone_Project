import React from 'react';

export default function Legend() {
  return (
    <div style={styles.container}>
      <div style={styles.row}>
        <Item color="#e2e8f0" shape="house" label="Base" />
        <Item color="#f59e0b" shape="square" label="Bakery" />
        <Item color="#3b82f6" shape="circle" label="Customer" />
        <Item color="#6366f1" shape="diamond" label="Drone" />
      </div>
      <div style={styles.row}>
        <span style={styles.hint}>Priority:</span>
        <Chip color="#3b82f6" label="Low" />
        <Chip color="#f59e0b" label="Med" />
        <Chip color="#ef4444" label="High" />
      </div>
    </div>
  );
}

function Item({ color, label }) {
  return (
    <div style={styles.item}>
      <div style={{ ...styles.dot, background: color }} />
      <span style={styles.label}>{label}</span>
    </div>
  );
}

function Chip({ color, label }) {
  return (
    <div style={styles.chip}>
      <div style={{ width: 6, height: 6, borderRadius: '50%', background: color }} />
      <span style={styles.chipLabel}>{label}</span>
    </div>
  );
}

const styles = {
  container: {
    position: 'absolute',
    bottom: 12,
    left: 12,
    background: '#12121eee',
    border: '1px solid #2d2d4a',
    borderRadius: 8,
    padding: '8px 12px',
    backdropFilter: 'blur(8px)',
    display: 'flex',
    flexDirection: 'column',
    gap: 6,
    zIndex: 10,
  },
  row: {
    display: 'flex',
    alignItems: 'center',
    gap: 10,
  },
  item: {
    display: 'flex',
    alignItems: 'center',
    gap: 5,
  },
  dot: {
    width: 8,
    height: 8,
    borderRadius: 2,
  },
  label: {
    fontSize: 10,
    color: '#94a3b8',
  },
  hint: {
    fontSize: 9,
    color: '#64748b',
  },
  chip: {
    display: 'flex',
    alignItems: 'center',
    gap: 3,
  },
  chipLabel: {
    fontSize: 9,
    color: '#94a3b8',
  },
};
