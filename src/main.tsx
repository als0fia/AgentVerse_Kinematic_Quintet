/**
 * @file src/main.tsx
 * @description Application Entry Point for the Antenna Swarm Control Console.
 *
 * This module initializes the React 19 root container, imports the global Tailwind v4 stylesheet
 * (`src/index.css`), logs the initial application bootstrap event, and mounts the `App` component.
 *
 * Use Cases:
 * 1. Mounting the interactive multi-robot paddy field UI into `#root`.
 * 2. Initializing structured diagnostic logging on startup.
 */

import { createRoot } from 'react-dom/client';
import App from './App.tsx';
import './index.css';
import { logFunctionCall } from './utils/logger.ts';

logFunctionCall('bootstrapAntennaConsole', { rootElementId: 'root' });

createRoot(document.getElementById('root')!).render(<App />);
