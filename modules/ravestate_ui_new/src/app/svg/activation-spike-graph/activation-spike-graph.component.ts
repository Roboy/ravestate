import { Component, OnDestroy } from '@angular/core';
import { Subscription } from "rxjs";

import { MockDataService } from "../../mock-data/mock-data.service";
import { NodeType } from "../elements/node.component";
import { DomSanitizer } from "@angular/platform-browser";

@Component({
    selector: 'app-activation-spike-graph',
    template: `
        <svg [attr.viewBox]="'0 0 ' + graphWidth + ' 1'" preserveAspectRatio="xMidYMid meet">
            <g class="moving-container" [style.transform]="getTransform()">
                <g connector *ngFor="let c of connectors" [fromX]="c.fromX" [toX]="c.toX" [fromY]="c.fromY" [toY]="c.toY"></g>
                <g node *ngFor="let node of nodes" [x]="node.x" [y]="node.y" [label]="node.label" [nodeType]="node.type"></g>
            </g>
        </svg>
        <div class="controls">
            {{(scale * 100).toFixed(0)}} % 
            <button (click)="scaleDown()" class="round">-</button>
            <button (click)="scaleUp()" class="round">+</button>
            <button (click)="scale = 1">100%</button> 
            |
            <button (click)="clear()">Clear</button>
        </div>
    `,
    styleUrls: ['./activation-spike-graph.component.scss']
})
export class ActivationSpikeGraphComponent implements OnDestroy {

    private subscriptions: Subscription;

    // internal graph size, defines the SVG coordinate system using viewBox
    graphWidth = 1000;

    // distance between two nodes (x-axis)
    nodeSpacingX = 150;

    nodes: Array<{x: number, y: number, label: string, type: NodeType}> = [];
    connectors: Array<{fromX: number, toX: number, fromY: number, toY: number}> = [];
    containerPosX = 0;  // translate whole container with a nice transition animation
    scale = 1;

    lastX = 900;

    constructor(private mockDataService: MockDataService, private sanitizer: DomSanitizer) {
        this.subscriptions = new Subscription();

        this.subscriptions.add(this.mockDataService.spikes.subscribe(spike => {
            const x = this.lastX;
            const y = (Math.random() * 2 - 1) * 200;
            const lastNode = this.nodes.length > 0 ? this.nodes[this.nodes.length - 1] : null;
            this.addNode(x, y, NodeType.SPIKE, 'spike', lastNode);
            this.containerPosX -= this.nodeSpacingX;
            this.lastX += this.nodeSpacingX;

        }));

        this.subscriptions.add(this.mockDataService.activations.subscribe(activation => {
            const x = this.lastX;
            const y = (Math.random() * 2 - 1) * 200;
            const lastNode = this.nodes.length > 0 ? this.nodes[this.nodes.length - 1] : null;
            this.addNode(x, y - 70, NodeType.ACTIVATION, 'activation 1', lastNode);
            this.addNode(x, y, NodeType.ACTIVATION, 'activation 2', lastNode);
            this.addNode(x, y + 70, NodeType.ACTIVATION, 'activation 3', lastNode);
            this.containerPosX -= this.nodeSpacingX;
            this.lastX += this.nodeSpacingX;
        }));
    }

    ngOnDestroy(): void {
        this.subscriptions.unsubscribe();
    }

    clear() {
        this.containerPosX = 0;
        this.lastX = 900;
        this.nodes = [];
        this.connectors = [];
    }

    scaleUp() {
        if (this.scale < 4) {
            this.scale *= 1.1;
        }
    }

    scaleDown() {
        if (this.scale >= 0.3) {
            this.scale /= 1.1;
        }
    }

    getTransform() {
        const transform = `translateX(900px) scale(${this.scale.toFixed(2)}) translateX(-900px) translateX(${this.containerPosX}px)`;
        return this.sanitizer.bypassSecurityTrustStyle(transform);
    }

    addNode(x: number, y: number, type: NodeType, label: string, connectTo?: {x: number, y: number}) {
        this.nodes.push({x, y, label, type});
        if (connectTo) {
            this.connectors.push({
                fromX: connectTo.x, toX: x,
                fromY: connectTo.y, toY: y
            });
        }
    }
}
