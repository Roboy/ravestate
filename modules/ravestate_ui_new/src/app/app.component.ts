import { Component } from '@angular/core';

@Component({
    selector: 'app-root',
    template: `
        <div style="text-align:center">
            <h1>
                Welcome to {{title}}!
            </h1>
        </div>
    `,
    styles: []
})
export class AppComponent {
    title = 'ravestate-ui';
}
