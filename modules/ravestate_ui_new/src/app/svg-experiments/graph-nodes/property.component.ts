import { Component } from '@angular/core';

@Component({
    selector: '[property-node]',
    template: `
        <svg:circle cx="0" cy="0" r="20"/>
    `,
    styleUrls: ['./node-styles.scss']
})
export class PropertyComponent {

    constructor() { }

}
