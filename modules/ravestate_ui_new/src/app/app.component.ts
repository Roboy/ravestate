import { Component } from '@angular/core';

@Component({
    selector: 'app-root',
    template: `
        <div class="root-container">
            <div class="header">
                Ravestate UI
            </div>
            <div class="main">
                <app-activation-spike-graph class="graph-area"></app-activation-spike-graph>
            </div>
            <div class="side">
                <app-mock-data-controller></app-mock-data-controller>
                <br>
                <div>
                    TODOs:
                    <ul>
                        <li>allow to move columns up and down (to a certain degree)</li>
                        <li>connect to real data</li>
                    </ul>
                </div>
                
            </div>
        </div>
    `,
    styleUrls: ['./app.component.scss']
})
export class AppComponent {
}
