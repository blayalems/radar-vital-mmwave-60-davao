import { Component, inject, OnInit } from '@angular/core';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';

// Angular Material 3 modules
import { MatCardModule } from '@angular/material/card';
import { MatButtonModule } from '@angular/material/button';
import { MatIconModule } from '@angular/material/icon';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';
import { MatExpansionModule } from '@angular/material/expansion';

import { StateService } from '../../services/state.service';
import { ApiService } from '../../services/api.service';

interface HelpTopic {
  title: string;
  category: 'general' | 'steps' | 'errors' | 'shortcuts';
  content: string;
}

@Component({
  selector: 'app-help',
  standalone: true,
  imports: [
    CommonModule,
    FormsModule,
    MatCardModule,
    MatButtonModule,
    MatIconModule,
    MatFormFieldModule,
    MatInputModule,
    MatExpansionModule
  ],
  templateUrl: './help.component.html',
  styleUrl: './help.component.css'
})
export class HelpComponent implements OnInit {
  protected readonly state = inject(StateService);
  protected readonly api = inject(ApiService);

  searchQuery = '';
  activeCategory: 'all' | 'general' | 'steps' | 'errors' | 'shortcuts' = 'all';

  helpTopics: HelpTopic[] = [
    {
      title: 'How does mmWave Radar telemetry work?',
      category: 'general',
      content: 'The 60 GHz mmWave radar module uses frequency-modulated continuous wave (FMCW) technology to sense physiological movements without touching the skin. It captures tiny phase changes (down to microns) caused by the heartbeat and breathing expansion of the chest wall. Advanced DSP algorithms isolate these movements from background clutter.'
    },
    {
      title: 'Active Sandbox Mode Explained',
      category: 'general',
      content: 'If the dashboard cannot reach the local Python trainer control server, it automatically activates Sandbox Mode. In this mode, telemetry data is simulated locally within the browser, allowing you to preview dashboards, reviews, haptic tests, and layout switches with full operational features. No actual USB hardware is polled.'
    },
    {
      title: 'Quick steps: Setting up a recording',
      category: 'steps',
      content: '1. Navigate to the Home workspace.\n2. Ensure the USB radar module is connected and check the active serial port dropdown list.\n3. Position the subject directly facing the radar sweep scope at a distance of 40 to 120 cm.\n4. Enter the subject ID and click "Start Telemetry Session".\n5. Direct the subject to sit completely still during the 3-second countdown phase to calibrate the baseline.'
    },
    {
      title: 'Troubleshooting: Jitter & Mismatch warnings',
      category: 'errors',
      content: 'If a "Physiological Mismatch Alert" is triggered, it indicates a high variance (drift) between the radar reported HR/RR and the connected BLE reference oximeter. Ensure the BLE oximeter is securely attached to the finger and the subject is not coughing or speaking. Real-time drift alerts emit audio warnings.'
    },
    {
      title: 'Operator Keyboard Shortcuts list',
      category: 'shortcuts',
      content: 'Maximize workflow speeds with keyboard controls:\n- "Ctrl + K": Open the floating interactive Command Palette.\n- "Alt + 1": Switch to live Overview dashboard.\n- "Alt + 2": Switch to real-time Waveforms view.\n- "Alt + 3": Switch to heart rate trend chart.\n- "Alt + 4": Switch to respiration trend graph.\n- "Alt + 5": Open session Snapshots vault.'
    }
  ];

  ngOnInit() {
    this.fetchHelpSchema();
  }

  async fetchHelpSchema() {
    try {
      const resp = await this.api.request('/api/help/schema');
      if (resp && resp.faq) {
        // Option to dynamically expand schemas
      }
    } catch (_) {}
  }

  setCategory(cat: 'all' | 'general' | 'steps' | 'errors' | 'shortcuts') {
    this.activeCategory = cat;
    this.state.triggerHaptic('tap');
  }

  getFilteredTopics(): HelpTopic[] {
    let list = this.helpTopics;
    if (this.activeCategory !== 'all') {
      list = list.filter(t => t.category === this.activeCategory);
    }
    if (this.searchQuery.trim()) {
      const q = this.searchQuery.toLowerCase().trim();
      list = list.filter(t => 
        t.title.toLowerCase().includes(q) || 
        t.content.toLowerCase().includes(q)
      );
    }
    return list;
  }
}
