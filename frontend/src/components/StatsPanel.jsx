import React from 'react';

const DRONE_COLORS = ['#6366f1', '#ec4899', '#f59e0b', '#22c55e', '#06b6d4', '#f97316', '#8b5cf6'];

export default function StatsPanel({ state, selected, onRemoveCustomer }) {
  if (!state) return null;

  const { bakeries, customers, drones, total_delivered = 0 } = state;
  const activeDrones = drones.filter(d => !d.is_idle).length;

  return (
    <div style={styles.container}>
      <div style={styles.section}>
        <div style={styles.statsRow}>
          <Stat label="Customers" value={customers.length} color="#3b82f6" />
          <Stat label="Drones" value={`${activeDrones}/${drones.length}`} color="#22c55e" />
          <Stat label="Delivered" value={total_delivered} color="#f59e0b" />
        </div>
      </div>

      <div style={styles.section}>
        <div style={styles.sectionTitle}>Bakeries</div>
        {bakeries.map(b => {
          const ratio = b.current_inventory / b.capacity;
          const color = ratio > 0.5 ? '#22c55e' : ratio > 0.2 ? '#f59e0b' : '#ef4444';
          return (
            <div key={b.id} style={styles.barRow}>
              <span style={styles.barLabel}>B{b.id}</span>
              <div style={styles.barTrack}>
                <div style={{ ...styles.barFill, width: `${ratio * 100}%`, background: color }} />
              </div>
              <span style={styles.barValue}>{b.current_inventory}/{b.capacity}</span>
            </div>
          );
        })}
      </div>

      {selected && (
        <div style={styles.section}>
          <div style={styles.sectionTitle}>
            {selected.type === 'drone' && `Drone ${selected.data.id}`}
            {selected.type === 'bakery' && `Bakery ${selected.data.id}`}
            {selected.type === 'customer' && `Customer ${selected.data.id}`}
          </div>

          {selected.type === 'drone' && (
            <div style={styles.detailList}>
              <Detail label="Status" value={selected.data.is_idle ? 'Idle' : 'Active'} />
              <Detail label="Load" value={`${selected.data.current_load}/${selected.data.max_capacity}`} />
              <Detail label="Position" value={`(${selected.data.pos.x.toFixed(1)}, ${selected.data.pos.y.toFixed(1)})`} />
              {selected.data.planned_route.length > 0 && (
                <div style={styles.routeList}>
                  {selected.data.planned_route.map((node, i) => (
                    <div key={i} style={{
                      ...styles.routeItem,
                      opacity: i < selected.data.route_progress ? 0.35 : 1
                    }}>
                      <span style={{
                        ...styles.routeDot,
                        background: node.type === 'pickup' ? '#f59e0b' : '#3b82f6'
                      }} />
                      <span style={styles.routeText}>
                        {node.type === 'pickup'
                          ? `Pick up ${node.bread_amount} from B${node.entity_id}`
                          : `Deliver ${node.bread_amount} to C${node.entity_id}`}
                        {node.committed ? ' (done)' : ''}
                      </span>
                    </div>
                  ))}
                </div>
              )}
            </div>
          )}

          {selected.type === 'bakery' && (
            <div style={styles.detailList}>
              <Detail label="Stock" value={`${selected.data.current_inventory}/${selected.data.capacity}`} />
              <Detail label="Position" value={`(${selected.data.pos.x.toFixed(1)}, ${selected.data.pos.y.toFixed(1)})`} />
            </div>
          )}

          {selected.type === 'customer' && (
            <div style={styles.detailList}>
              <Detail label="Order" value={`${selected.data.order_quantity} loaves`} />
              <Detail label="Priority" value={selected.data.priority_weight.toFixed(1)} />
              <Detail label="Position" value={`(${selected.data.pos.x.toFixed(1)}, ${selected.data.pos.y.toFixed(1)})`} />
              <button style={styles.removeBtn} onClick={() => onRemoveCustomer(selected.data.id)}>
                Remove
              </button>
            </div>
          )}
        </div>
      )}

      <div style={styles.section}>
        <div style={styles.sectionTitle}>Drones</div>
        {drones.map((d, i) => (
          <div key={d.id} style={styles.droneRow}>
            <span style={{ ...styles.droneDot, background: d.is_idle ? '#475569' : DRONE_COLORS[i % DRONE_COLORS.length] }} />
            <span style={styles.droneName}>D{d.id}</span>
            <span style={styles.droneStatus}>
              {d.is_idle ? 'idle' : `${d.current_load}/${d.max_capacity}`}
            </span>
            {d.planned_route.length > 0 && (
              <span style={styles.droneStops}>{d.planned_route.length} stops</span>
            )}
          </div>
        ))}
      </div>
    </div>
  );
}

function Stat({ label, value, color }) {
  return (
    <div style={styles.statCard}>
      <div style={{ ...styles.statValue, color }}>{value}</div>
      <div style={styles.statLabel}>{label}</div>
    </div>
  );
}

function Detail({ label, value }) {
  return (
    <div style={styles.detailRow}>
      <span style={styles.detailLabel}>{label}</span>
      <span style={styles.detailValue}>{value}</span>
    </div>
  );
}

const styles = {
  container: {
    flex: 1,
    overflowY: 'auto',
    padding: '0 0 12px 0',
  },
  section: {
    padding: '10px 20px',
    borderBottom: '1px solid #1e1e32',
  },
  sectionTitle: {
    fontSize: 11,
    fontWeight: 600,
    color: '#64748b',
    textTransform: 'uppercase',
    letterSpacing: '0.5px',
    marginBottom: 8,
  },
  statsRow: {
    display: 'flex',
    gap: 8,
  },
  statCard: {
    flex: 1,
    background: '#12121e',
    borderRadius: 6,
    padding: '8px 6px',
    textAlign: 'center',
    border: '1px solid #1e1e32',
  },
  statValue: {
    fontSize: 18,
    fontWeight: 700,
    fontFamily: "'JetBrains Mono', monospace",
    lineHeight: 1,
    marginBottom: 3,
  },
  statLabel: {
    fontSize: 9,
    color: '#64748b',
    textTransform: 'uppercase',
    letterSpacing: '0.3px',
  },
  barRow: {
    display: 'flex',
    alignItems: 'center',
    gap: 8,
    marginBottom: 5,
  },
  barLabel: {
    fontSize: 11,
    color: '#94a3b8',
    fontFamily: "'JetBrains Mono', monospace",
    minWidth: 22,
  },
  barTrack: {
    flex: 1,
    height: 5,
    background: '#1e1e32',
    borderRadius: 3,
    overflow: 'hidden',
  },
  barFill: {
    height: '100%',
    borderRadius: 3,
    transition: 'width 0.3s ease',
  },
  barValue: {
    fontSize: 10,
    color: '#94a3b8',
    fontFamily: "'JetBrains Mono', monospace",
    minWidth: 36,
    textAlign: 'right',
  },
  detailList: {
    display: 'flex',
    flexDirection: 'column',
    gap: 3,
  },
  detailRow: {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '2px 0',
  },
  detailLabel: {
    fontSize: 12,
    color: '#64748b',
  },
  detailValue: {
    fontSize: 12,
    color: '#e2e8f0',
    fontFamily: "'JetBrains Mono', monospace",
  },
  routeList: {
    marginTop: 4,
    display: 'flex',
    flexDirection: 'column',
    gap: 2,
  },
  routeItem: {
    display: 'flex',
    alignItems: 'center',
    gap: 6,
    padding: '2px 0',
  },
  routeDot: {
    width: 5,
    height: 5,
    borderRadius: '50%',
    flexShrink: 0,
  },
  routeText: {
    fontSize: 11,
    color: '#94a3b8',
  },
  removeBtn: {
    marginTop: 6,
    padding: '6px 12px',
    background: 'transparent',
    color: '#ef4444',
    border: '1px solid #ef444440',
    borderRadius: 6,
    cursor: 'pointer',
    fontSize: 12,
    fontFamily: 'inherit',
    fontWeight: 500,
    width: '100%',
  },
  droneRow: {
    display: 'flex',
    alignItems: 'center',
    gap: 8,
    padding: '3px 0',
    fontSize: 12,
  },
  droneDot: {
    width: 6,
    height: 6,
    borderRadius: '50%',
    flexShrink: 0,
  },
  droneName: {
    color: '#e2e8f0',
    fontFamily: "'JetBrains Mono', monospace",
    fontWeight: 500,
    minWidth: 24,
  },
  droneStatus: {
    color: '#64748b',
    flex: 1,
  },
  droneStops: {
    color: '#475569',
    fontSize: 11,
  },
};
