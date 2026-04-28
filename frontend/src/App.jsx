import React, { useState, useEffect, useCallback, useRef } from 'react';
import SimulationMap from './components/SimulationMap';
import ControlPanel from './components/ControlPanel';
import StatsPanel from './components/StatsPanel';
import EventLog from './components/EventLog';
import AddCustomerModal from './components/AddCustomerModal';
import { fetchState, stepRound, addCustomer, removeCustomer, resetSimulation } from './utils/api';

export default function App() {
  const [state, setState] = useState(null);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(800);
  const [selected, setSelected] = useState(null);
  const [events, setEvents] = useState([]);
  const [error, setError] = useState(null);
  const [showAddCustomer, setShowAddCustomer] = useState(false);
  const [clickPos, setClickPos] = useState(null);
  const [connected, setConnected] = useState(false);
  const intervalRef = useRef(null);
  const prevStateRef = useRef(null);

  useEffect(() => {
    fetchState()
      .then(data => {
        setState(data);
        setConnected(true);
        setEvents([{ round: data.round, text: 'Simulation ready', type: 'system' }]);
      })
      .catch(() => {
        setError('Cannot connect to backend. Start the server with: ./bread_delivery --server');
      });
  }, []);

  const generateEvents = useCallback((newState, prevState) => {
    if (!newState || !prevState) return [];
    const newEvents = [];
    const r = newState.round;

    if (newState.resolved_intents) {
      for (const intent of newState.resolved_intents) {
        newEvents.push({
          round: r,
          text: `D${intent.drone_id} picking up ${intent.requested_bread_amount} from B${intent.bakery_id} for C${intent.customer_id}`,
          type: 'delivery'
        });
      }
    }

    if (newState.customers && prevState.customers) {
      const prevIds = new Set(prevState.customers.map(c => c.id));
      for (const c of newState.customers) {
        if (!prevIds.has(c.id)) {
          newEvents.push({
            round: r,
            text: `New customer C${c.id} wants ${c.order_quantity} loaves`,
            type: 'customer'
          });
        }
      }

      const prevServed = new Set(newState.customers.map(c => c.id));
      for (const c of prevState.customers) {
        if (!prevServed.has(c.id)) {
          newEvents.push({
            round: r,
            text: `C${c.id} served and removed`,
            type: 'served'
          });
        }
      }
    }

    return newEvents;
  }, []);

  const doStep = useCallback(async () => {
    try {
      const newState = await stepRound();
      const newEvents = generateEvents(newState, prevStateRef.current || state);
      prevStateRef.current = state;
      setState(newState);
      if (newEvents.length > 0) {
        setEvents(prev => [...newEvents, ...prev].slice(0, 100));
      }
      if (!newState.continued) {
        setPlaying(false);
        setEvents(prev => [{ round: newState.round, text: 'Simulation complete', type: 'system' }, ...prev]);
      }
    } catch (err) {
      setError('Lost connection to server');
      setPlaying(false);
    }
  }, [state, generateEvents]);

  useEffect(() => {
    if (playing) {
      intervalRef.current = setInterval(doStep, speed);
    }
    return () => {
      if (intervalRef.current) clearInterval(intervalRef.current);
    };
  }, [playing, speed, doStep]);

  const handlePlayPause = useCallback(() => setPlaying(p => !p), []);
  const handleStep = useCallback(() => doStep(), [doStep]);

  const handleReset = useCallback(async () => {
    setPlaying(false);
    try {
      const newState = await resetSimulation();
      setState(newState);
      prevStateRef.current = null;
      setSelected(null);
      setEvents([{ round: 0, text: 'Simulation reset', type: 'system' }]);
    } catch (err) {
      setError('Failed to reset simulation');
    }
  }, []);

  const handleAddCustomer = useCallback(async (x, y, qty, name) => {
    await addCustomer(x, y, qty, name);
    const newState = await fetchState();
    setState(newState);
    setEvents(prev => [{
      round: newState.round,
      text: `Added customer at (${x.toFixed(0)}, ${y.toFixed(0)}) wanting ${qty} loaves`,
      type: 'user'
    }, ...prev]);
    setShowAddCustomer(false);
    setClickPos(null);
  }, []);

  const handleRemoveCustomer = useCallback(async (id) => {
    await removeCustomer(id);
    const newState = await fetchState();
    setState(newState);
    setEvents(prev => [{
      round: newState.round,
      text: `Removed customer C${id}`,
      type: 'user'
    }, ...prev]);
    setSelected(null);
  }, []);

  const handleMapClick = useCallback((simPos) => {
    setClickPos(simPos);
    setShowAddCustomer(true);
  }, []);

  if (error) {
    return (
      <div style={styles.errorScreen}>
        <div style={styles.errorTitle}>Connection Error</div>
        <div style={styles.errorMsg}>{error}</div>
        <div style={styles.errorHint}>
          <code style={styles.code}>cd backend && ./build/bread_delivery --server</code>
        </div>
      </div>
    );
  }

  if (!state) {
    return (
      <div style={styles.loadingScreen}>
        <div style={styles.spinner} />
        <div style={{ marginTop: 16 }}>Connecting...</div>
      </div>
    );
  }

  return (
    <div style={styles.app}>
      <div style={styles.topBar}>
        <div style={styles.logo}>Drone Delivery</div>
        <div style={styles.roundDisplay}>Round {state.round}</div>
        <div style={styles.topBarRight}>
          <div style={styles.connectionDot(connected)} />
          <span style={styles.topBarLabel}>{connected ? 'Live' : 'Offline'}</span>
        </div>
      </div>

      <div style={styles.main}>
        <div style={styles.mapSection}>
          <SimulationMap
            state={state}
            gridWidth={100}
            gridHeight={100}
            selected={selected}
            onSelect={setSelected}
            onMapClick={handleMapClick}
          />
        </div>

        <div style={styles.rightPanel}>
          <ControlPanel
            playing={playing}
            speed={speed}
            onPlayPause={handlePlayPause}
            onStep={handleStep}
            onSpeedChange={setSpeed}
            onAddCustomer={() => setShowAddCustomer(true)}
            onReset={handleReset}
          />

          <StatsPanel
            state={state}
            selected={selected}
            onRemoveCustomer={handleRemoveCustomer}
          />

          <EventLog events={events} />
        </div>
      </div>

      {showAddCustomer && (
        <AddCustomerModal
          initialPos={clickPos}
          gridWidth={100}
          gridHeight={100}
          onAdd={handleAddCustomer}
          onClose={() => { setShowAddCustomer(false); setClickPos(null); }}
        />
      )}
    </div>
  );
}

const styles = {
  app: {
    display: 'flex',
    flexDirection: 'column',
    height: '100vh',
    background: '#0a0a0f',
    overflow: 'hidden',
  },
  topBar: {
    display: 'flex',
    alignItems: 'center',
    justifyContent: 'space-between',
    padding: '10px 24px',
    background: '#101018',
    borderBottom: '1px solid #1e1e32',
    minHeight: 48,
  },
  logo: {
    fontSize: 15,
    fontWeight: 600,
    color: '#e2e8f0',
    letterSpacing: '-0.3px',
  },
  roundDisplay: {
    fontSize: 18,
    fontWeight: 700,
    color: '#6366f1',
    fontFamily: "'JetBrains Mono', monospace",
  },
  topBarRight: {
    display: 'flex',
    alignItems: 'center',
    gap: 6,
  },
  connectionDot: (connected) => ({
    width: 7,
    height: 7,
    borderRadius: '50%',
    background: connected ? '#22c55e' : '#ef4444',
  }),
  topBarLabel: {
    fontSize: 11,
    color: '#94a3b8',
  },
  main: {
    display: 'flex',
    flex: 1,
    overflow: 'hidden',
  },
  mapSection: {
    flex: 1,
    position: 'relative',
    minWidth: 0,
  },
  rightPanel: {
    width: 360,
    display: 'flex',
    flexDirection: 'column',
    background: '#0f0f18',
    borderLeft: '1px solid #1e1e32',
    overflow: 'hidden',
  },
  errorScreen: {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center',
    height: '100vh',
    background: '#0a0a0f',
    gap: 10,
  },
  errorTitle: {
    fontSize: 18,
    fontWeight: 600,
    color: '#f1f5f9',
  },
  errorMsg: {
    fontSize: 13,
    color: '#94a3b8',
    textAlign: 'center',
    maxWidth: 400,
  },
  errorHint: {
    fontSize: 12,
    color: '#64748b',
    marginTop: 8,
  },
  code: {
    background: '#1e1e32',
    padding: '3px 10px',
    borderRadius: 4,
    fontFamily: "'JetBrains Mono', monospace",
    fontSize: 11,
  },
  loadingScreen: {
    display: 'flex',
    flexDirection: 'column',
    alignItems: 'center',
    justifyContent: 'center',
    height: '100vh',
    background: '#0a0a0f',
    color: '#94a3b8',
    fontSize: 13,
  },
  spinner: {
    width: 28,
    height: 28,
    border: '3px solid #1e1e32',
    borderTopColor: '#6366f1',
    borderRadius: '50%',
    animation: 'spin 0.8s linear infinite',
  },
};
