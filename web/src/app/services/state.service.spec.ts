import { TestBed } from '@angular/core/testing';
import { StateService, DEFAULT_KPI_THRESHOLDS } from './state.service';
import { PersistenceService } from './persistence.service';

describe('StateService', () => {
  let service: StateService;
  let mockPersistence: Partial<PersistenceService>;

  beforeEach(() => {
    mockPersistence = {
      quarantineLegacyLocalStorage: vi.fn().mockResolvedValue(undefined),
      get: vi.fn().mockResolvedValue(undefined),
      put: vi.fn().mockResolvedValue(undefined)
    };

    TestBed.configureTestingModule({
      providers: [
        StateService,
        { provide: PersistenceService, useValue: mockPersistence }
      ]
    });

    service = TestBed.inject(StateService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });

  it('should initialize with default states', () => {
    expect(service.currentView()).toBe('live');
    expect(service.activeTab()).toBe('tab-overview');
    expect(service.ctlOn()).toBe(false);
    expect(service.paused()).toBe(false);
    // The Radar Vital Redesign prototype defaults to the light theme.
    expect(service.theme()).toBe('light');
    expect(service.kpiThresholds()).toEqual(DEFAULT_KPI_THRESHOLDS);
  });

  it('should update theme and store it', () => {
    service.theme.set('night');
    expect(service.theme()).toBe('night');
  });

  it('should handle alert updates', () => {
    service.pushAlert('Test warning', 'warn', 'unit-test');
    const alerts = service.alertHistory();
    expect(alerts.length).toBe(1);
    expect(alerts[0].msg).toBe('Test warning');
    expect(alerts[0].severity).toBe('warn');
    expect(alerts[0].source).toBe('unit-test');
  });

  it('should toggle and manage alert pins', () => {
    const alertId = 'alert_12345';
    service.toggleAlertPin(alertId);
    expect(service.alertPins()).toContain(alertId);

    service.toggleAlertPin(alertId);
    expect(service.alertPins()).not.toContain(alertId);
  });

  it('should dismiss alerts and unpin them', () => {
    const alertId = 'alert_99999';
    service.alertHistory.set([{ id: alertId, ts: Date.now(), msg: 'Test warning', severity: 'warn', source: 'telemetry' }]);
    service.alertPins.set([alertId]);

    service.dismissAlert(alertId);
    expect(service.alertHistory()[0].dismissed).toBe(true);
    expect(service.alertPins()).not.toContain(alertId);
  });

  it('should snooze alerts', () => {
    const alertId = 'alert_55555';
    service.alertHistory.set([{ id: alertId, ts: Date.now(), msg: 'Test warning', severity: 'warn', source: 'telemetry' }]);
    
    service.snoozeAlert(alertId, 5);
    expect(service.alertHistory()[0].snoozedUntil).toBeGreaterThan(Date.now());
  });

  it('should respect haptic feedback mute triggers', () => {
    if (!('vibrate' in navigator)) {
      (navigator as any).vibrate = vi.fn();
    }
    const vibrateSpy = vi.spyOn(navigator, 'vibrate').mockImplementation(() => true);
    
    service.hxMode.set('off');
    service.triggerHaptic('tap');
    expect(vibrateSpy).not.toHaveBeenCalled();

    service.hxMode.set('on');
    service.triggerHaptic('tap');
    expect(vibrateSpy).toHaveBeenCalled();
    
    vibrateSpy.mockRestore();
  });
});
