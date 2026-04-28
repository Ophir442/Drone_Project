/**
 * Map simulation coordinates to canvas pixel coordinates.
 */
export function simToCanvas(simX, simY, canvasWidth, canvasHeight, gridWidth, gridHeight, padding = 50) {
  const drawWidth = canvasWidth - 2 * padding;
  const drawHeight = canvasHeight - 2 * padding;

  const x = padding + (simX / gridWidth) * drawWidth;
  const y = padding + ((gridHeight - simY) / gridHeight) * drawHeight; // flip Y for screen coords
  return { x, y };
}

/**
 * Map canvas pixel coordinates back to simulation coordinates.
 */
export function canvasToSim(canvasX, canvasY, canvasWidth, canvasHeight, gridWidth, gridHeight, padding = 50) {
  const drawWidth = canvasWidth - 2 * padding;
  const drawHeight = canvasHeight - 2 * padding;

  const simX = ((canvasX - padding) / drawWidth) * gridWidth;
  const simY = gridHeight - ((canvasY - padding) / drawHeight) * gridHeight;
  return {
    x: Math.max(0, Math.min(gridWidth, simX)),
    y: Math.max(0, Math.min(gridHeight, simY))
  };
}

/**
 * Format a position for display.
 */
export function formatPos(pos) {
  if (!pos) return '(?, ?)';
  return `(${pos.x.toFixed(1)}, ${pos.y.toFixed(1)})`;
}
