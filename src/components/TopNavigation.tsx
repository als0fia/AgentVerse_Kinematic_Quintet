/**
 * @file src/components/TopNavigation.tsx
 * @description Top Navigation Header for the Antenna Swarm Control Console.
 *
 * This component strictly implements the three-zone Top Bar Contract:
 * - Zone 1 (Brand): Single-line wordmark (`Antenna`) in crisp display typography.
 * - Zone 2 (Navigation Links): Concise single-line navigation links for switching workspace views
 *   (`Field Map`, `Robot Plans`, `Fault Triage`, `Human Teleop`).
 * - Zone 3 (Primary Action): High-priority Global Fleet Emergency Stop / Resume control button.
 *
 * Use Cases:
 * 1. Instant workspace view switching without cluttering the header with secondary badges or tickers.
 * 2. One-click Global Emergency Stop (E-Stop) accessible from any screen in the console.
 */

import React from 'react';
import { APP_CONFIG } from '../config/appConfig';
import { logFunctionCall } from '../utils/logger';

export type WorkspaceTab = 'FIELD_OPS' | 'ROBOT_PLANS' | 'FAULT_TRIAGE' | 'HUMAN_TELEOP';

interface TopNavigationProps {
  activeTab: WorkspaceTab;
  onSelectTab: (tab: WorkspaceTab) => void;
  fleetEmergencyStopActive: boolean;
  onToggleFleetEmergencyStop: () => void;
}

/**
 * Renders the top navigation bar with brand title, workspace links, and Global Emergency Stop CTA.
 * @param props - TopNavigationProps configuration and callbacks.
 * @returns React JSX header element.
 */
export const TopNavigation: React.FC<TopNavigationProps> = ({
  activeTab,
  onSelectTab,
  fleetEmergencyStopActive,
  onToggleFleetEmergencyStop,
}) => {
  /**
   * Handles navigation link clicks and logs the selected workspace tab.
   * @param tab - Target WorkspaceTab identifier.
   */
  function handleNavClick(tab: WorkspaceTab): void {
    logFunctionCall('TopNavigation.handleNavClick', { tab });
    onSelectTab(tab);
  }

  /**
   * Handles clicking the primary Global Emergency Stop / Resume action button.
   */
  function handleEmergencyButtonClick(): void {
    logFunctionCall('TopNavigation.handleEmergencyButtonClick', {
      nextState: !fleetEmergencyStopActive,
    });
    onToggleFleetEmergencyStop();
  }

  const navItems: { id: WorkspaceTab; label: string }[] = [
    { id: 'FIELD_OPS', label: 'Field Operations' },
    { id: 'ROBOT_PLANS', label: 'Robot Plans' },
    { id: 'FAULT_TRIAGE', label: 'Fault Triage' },
    { id: 'HUMAN_TELEOP', label: 'Human Control' },
  ];

  return (
    <header className="flex items-center justify-between gap-8 px-6 py-3.5 bg-slate-950 border-b border-slate-800">
      {/* Zone 1: Single text element wordmark */}
      <a
        href="#top"
        onClick={(e) => {
          e.preventDefault();
          handleNavClick('FIELD_OPS');
        }}
        className="text-lg font-bold tracking-tight text-slate-100 whitespace-nowrap shrink-0 focus-visible:outline-2 focus-visible:outline-cyan-400"
      >
        {APP_CONFIG.appTitle}
      </a>

      {/* Zone 2: 4 clean single-line text navigation links */}
      <nav aria-label="Primary Workspace Navigation" className="flex items-center gap-6 text-sm font-medium">
        {navItems.map((item) => {
          const isActive = activeTab === item.id;
          return (
            <button
              key={item.id}
              type="button"
              onClick={() => handleNavClick(item.id)}
              className={`whitespace-nowrap shrink-0 py-1 transition-colors border-b-2 focus-visible:outline-2 focus-visible:outline-cyan-400 ${
                isActive
                  ? 'border-cyan-400 text-slate-100 font-semibold'
                  : 'border-transparent text-slate-400 hover:text-slate-200'
              }`}
            >
              {item.label}
            </button>
          );
        })}
      </nav>

      {/* Zone 3: 1 primary action (Global Emergency Stop / Resume) */}
      <div className="flex items-center shrink-0">
        <button
          type="button"
          onClick={handleEmergencyButtonClick}
          className={`px-4 py-2 text-xs font-semibold rounded-md transition-colors whitespace-nowrap shrink-0 focus-visible:outline-2 focus-visible:outline-offset-2 ${
            fleetEmergencyStopActive
              ? 'bg-emerald-600 hover:bg-emerald-500 text-white focus-visible:outline-emerald-400'
              : 'bg-rose-600 hover:bg-rose-500 text-white focus-visible:outline-rose-400'
          }`}
        >
          {fleetEmergencyStopActive ? 'Resume Autonomous Swarm' : 'Emergency Fleet Stop'}
        </button>
      </div>
    </header>
  );
};
