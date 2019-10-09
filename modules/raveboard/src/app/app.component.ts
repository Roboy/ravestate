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
                <app-mock-data-controller></app-mock-data-controller>
                <br>
                <div>
                    TODOs:
                    <ul>
                        <li>allow to move graph with drag</li>
                        <li>idea: allow to move columns up and down (to a certain degree)</li>
                    </ul>
                </div>
            </div> 
            <div class="side2">
                <app-chat-window></app-chat-window>
            </div>
        </div>
    `,
    styleUrls: ['./app.component.scss']
})
export class AppComponent {
}
