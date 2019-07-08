import { Component } from '@angular/core';

@Component({
    selector: 'app-root',
    template: `
        <div class="root-container">
            <div class="header">
                Ravestate UI
            </div>
            <div class="main">
                <svg-experiments class="graph-area"></svg-experiments>
            </div>
            <div class="side">
                this is a cool little sidebar
            </div>
        </div>
    `,
    styleUrls: ['./app.component.scss']
})
export class AppComponent {
    title = 'ravestate-ui';
}
