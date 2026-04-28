const API_BASE = 'http://localhost:8080';

export async function fetchState() {
  const res = await fetch(`${API_BASE}/api/state`);
  return res.json();
}

export async function stepRound() {
  const res = await fetch(`${API_BASE}/api/step`, { method: 'POST' });
  return res.json();
}

export async function addCustomer(x, y, orderQuantity, name = '') {
  const res = await fetch(`${API_BASE}/api/customer`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ x, y, order_quantity: orderQuantity, name })
  });
  return res.json();
}

export async function removeCustomer(id) {
  const res = await fetch(`${API_BASE}/api/customer`, {
    method: 'DELETE',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ id })
  });
  return res.json();
}

export async function initializeSimulation() {
  const res = await fetch(`${API_BASE}/api/init`, { method: 'POST' });
  return res.json();
}

export async function resetSimulation() {
  const res = await fetch(`${API_BASE}/api/reset`, { method: 'POST' });
  return res.json();
}

export async function fetchConfig() {
  const res = await fetch(`${API_BASE}/api/config`);
  return res.json();
}
