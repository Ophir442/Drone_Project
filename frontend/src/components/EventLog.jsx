import React from 'react';

const TYPE_COLORS = {
  delivery: '#22c55e',
  customer: '#3b82f6',
  served: '#a78bfa',
  system: '#6366f1',
  user: '#ec4899',
};

export default function EventLog({ events }) {
  return (
    <div style={styles.container}>
      <div style={styles.header}>
        <span style={styles.title}>Events</span>
        <span style={styles.count}>{events.length}</span>
      </div>
      <div style={styles.list}>
        {events.length === 0 ? (
          <div style={styles.empty}>Press Play or Step to begin.</div>
        ) : (
          events.map((event, i) => (
            <div key={i} style={styles.event}>
              <span style={{ ...styles.eventDot, background: TYPE_COLORS[event.type] || '#475569' }} />
              <span style={styles.eventRound}>R{event.round}</span>
              <span style={styles.eventText}>{event.text}</span>
            </div>
          ))
        )}
      </div>
    </div>
  );
}

const styles = {
  container: {
    display: 'flex',
    flexDirection: 'column',
    minHeight: 0,
    flex: '0 0 180px',
    borderTop: '1px solid #1e1e32',
  },
  header: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: '8px 20px',
    borderBottom: '1px solid #1e1e32',
  },
  title: {
    fontSize: 11,
    fontWeight: 600,
    color: '#64748b',
    textTransform: 'uppercase',
    letterSpacing: '0.5px',
  },
  count: {
    fontSize: 10,
    color: '#475569',
    background: '#1e1e32',
    padding: '1px 7px',
    borderRadius: 10,
    fontFamily: "'JetBrains Mono', monospace",
  },
  list: {
    flex: 1,
    overflowY: 'auto',
    padding: '4px 12px',
  },
  empty: {
    fontSize: 12,
    color: '#475569',
    textAlign: 'center',
    padding: 16,
  },
  event: {
    display: 'flex',
    alignItems: 'flex-start',
    gap: 8,
    padding: '3px 8px',
    fontSize: 11,
    lineHeight: '1.4',
    borderBottom: '1px solid #12121e',
  },
  eventDot: {
    width: 5,
    height: 5,
    borderRadius: '50%',
    marginTop: 5,
    flexShrink: 0,
  },
  eventRound: {
    color: '#475569',
    fontFamily: "'JetBrains Mono', monospace",
    fontSize: 10,
    minWidth: 22,
    flexShrink: 0,
    marginTop: 1,
  },
  eventText: {
    color: '#94a3b8',
  },
};
