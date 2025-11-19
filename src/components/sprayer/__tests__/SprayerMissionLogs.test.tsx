import React from 'react';
import { render, screen, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import { vi, describe, it, expect, beforeEach } from 'vitest';

// We'll mock useRover to provide onMissionEvent registration
let registeredHandler: ((ev: any) => void) | null = null;
vi.mock('../../../context/RoverContext', () => ({
  useRover: () => ({
    onMissionEvent: (cb: (ev: any) => void) => {
      registeredHandler = cb;
    },
  }),
}));

import SprayerMissionLogs from '../SprayerMissionLogs';
import type { Waypoint } from '../../../types';

describe('SprayerMissionLogs realtime updates', () => {
  const waypoints: Waypoint[] = [
    { id: 1, lat: 13.07, lng: 80.26 },
    { id: 2, lat: 13.071, lng: 80.261 },
    { id: 3, lat: 13.072, lng: 80.262 },
  ] as any;

  const mockWaypointReached = {
    event_type: 'waypoint_reached',
    waypoint_id: 2,
    current_waypoint: 2,
    timestamp: '2025-11-18T16:30:45.123Z',
    position: { lat: 13.072, lng: 80.262, alt: 50 },
    message: 'Waypoint 2 reached successfully',
  };

  const mockWaypointMarked = {
    event_type: 'waypoint_marked',
    waypoint_id: 2,
    current_waypoint: 2,
    timestamp: '2025-11-18T16:30:50.456Z',
    marking_status: 'completed',
    spray_duration: 5.2,
    message: 'Waypoint 2 marking completed',
  };

  beforeEach(() => {
    registeredHandler = null;
  });

  it('updates WP_REACH and WP_MARK columns when mission events arrive', async () => {
    render(<SprayerMissionLogs waypoints={waypoints} />);

    // Initially, WP_REACH and WP_MARK for waypoint 2 should be '-'
    const rows = screen.getAllByRole('row');
    // header + 3 rows -> index 1..3 are data rows
    const wp2row = rows[2];
    expect(wp2row).toBeTruthy();

    const cells = wp2row.querySelectorAll('td');
    const reachCell = cells[4];
    const markCell = cells[5];

    expect(reachCell.textContent).toBe('-');
    expect(markCell.textContent).toBe('-');

    // Simulate waypoint_reached event
    expect(registeredHandler).not.toBeNull();
    registeredHandler && registeredHandler(mockWaypointReached);

    // After event, WP_REACH should update (non '-' text)
    await waitFor(() => {
      expect(reachCell.textContent).not.toBe('-');
    });

    // Now simulate waypoint_marked
    registeredHandler && registeredHandler(mockWaypointMarked);

    // WP_MARK should show 'completed'
    await waitFor(() => {
      expect(markCell.textContent).toBe('completed');
    });
  });
});
