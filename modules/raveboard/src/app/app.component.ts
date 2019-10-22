import { Component } from '@angular/core';

@Component({
    selector: 'app-root',
    template: `
        <div class="root-container">
            <div class="header">
                Raveboard 🏄
            </div>
            <div class="main">
                <app-activation-spike-graph class="graph-area"></app-activation-spike-graph>
            </div>
            <div class="side">                
                <app-chat-window [style.display]="debugMode ? 'none' : null"></app-chat-window>                
                <app-mock-data-controller [style.display]="debugMode ? null : 'none'"></app-mock-data-controller>                
            </div> 
            <!-- debug button, can be enabled with dev tools -->
            <div style="position: fixed; top: 5px; right: 5px; display: none">
                <button (click)="debugMode = !debugMode">Debug</button>
            </div>
        </div>
    `,
    styleUrls: ['./app.component.scss']
})
export class AppComponent {
    debugMode = false;
}
