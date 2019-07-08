import { Component } from '@angular/core';

@Component({
    selector: 'app-root',
    template: `
        <div class="root-container">
            <div class="header">
                Ravestate UI
            </div>
            <div class="main">
                <!-- <svg-experiments class="graph-area"></svg-experiments> -->
                <app-activation-spike-graph class="graph-area"></app-activation-spike-graph>
            </div>
            <div class="side">
                <app-mock-data-controller></app-mock-data-controller>
            </div>
        </div>
    `,
    styleUrls: ['./app.component.scss']
})
export class AppComponent {
    title = 'ravestate-ui';
}
