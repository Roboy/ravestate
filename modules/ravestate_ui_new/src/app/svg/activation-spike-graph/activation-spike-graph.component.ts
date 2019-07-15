import { Component, OnDestroy } from '@angular/core';
import { Subscription } from "rxjs";

import { MockDataService } from "../../mock-data/mock-data.service";
import { NodeType } from "../elements/node.component";

@Component({
    selector: 'app-activation-spike-graph',
    template: `
        <svg [attr.viewBox]="'0 0 ' + graphWidth + ' ' + graphHeight" preserveAspectRatio="xMidYMid meet">
            <g class="moving-container" [style.transform]="'translateX(' + (containerPosX)  + 'px)'">
                <g node *ngFor="let node of nodes" [x]="node.x" [y]="node.y" [label]="node.label"
                   [nodeType]="node.type"></g>
            </g>
        </svg>
        <div class="controls">
            <button (click)="clear()">Clear</button>
        </div>
    `,
    styleUrls: ['./activation-spike-graph.component.scss']
})
export class ActivationSpikeGraphComponent implements OnDestroy {

    private dataSub: Subscription;

    // internal graph size, defines the SVG coordinate system using viewBox
    graphWidth = 1000;
    graphHeight = 300;
    // distance between two nodes (x-axis)
    nodeSpacingX = 100;

    nodes: Array<{x: number, y: number, label: string, type: NodeType}> = [];
    containerPosX = 0;  // translate whole container with a nice transition animation

    constructor(private mockDataService: MockDataService) {
        this.dataSub = mockDataService.dataStream.subscribe(data => {
            const newNode = {
                x: this.graphWidth - this.containerPosX,
                y: Math.floor(Math.random() * this.graphHeight),
                label: 'node ' + this.nodes.length,
                type: this.nodes.length % 2 == 0 ? NodeType.SPIKE : NodeType.ACTIVATION
            };
            this.nodes.push(newNode);
            this.containerPosX -= this.nodeSpacingX;
        });
    }

    ngOnDestroy(): void {
        this.dataSub.unsubscribe();
    }

    clear() {
        this.containerPosX = 0;
        this.nodes = [];
    }
}
