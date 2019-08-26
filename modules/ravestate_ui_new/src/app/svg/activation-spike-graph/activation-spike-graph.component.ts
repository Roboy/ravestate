import { Component, OnDestroy } from '@angular/core';
import { Subscription } from "rxjs";

import { ActivationUpdate, MockDataService, SpikeUpdate } from "../../mock-data/mock-data.service";
import { NodeType } from "../elements/node.component";
import { DomSanitizer } from "@angular/platform-browser";

export class Node {
    id: number;
    column: number;

    element: SpikeUpdate | ActivationUpdate;
    parents: Array<Node>;

    x: number;
    y: number;
    label: string;

    constructor(element: SpikeUpdate | ActivationUpdate) {
        this.element = element;
        this.parents = [];
        if (element.type === 'activation') {
            this.label = `${element.state} [${element.id}]`;
            this.id = Node.activationID(element.id);
        } else {
            this.label = `${element.signal} [${element.id}]`;
            this.id = Node.spikeID(element.id);
        }
    }

    static activationID(id: number): number {
        return id * 2;
    }
    static spikeID(id: number): number {
        return id * 2 + 1;
    }
}


@Component({
    selector: 'app-activation-spike-graph',
    template: `
        <svg [attr.viewBox]="'0 0 ' + graphWidth + ' 1'" preserveAspectRatio="xMidYMid meet">
            <g class="moving-container" [style.transform]="getTransform()">

                <!-- connectors for every node -->
                <ng-container *ngFor="let node of allNodes.values()">
                    <g connector *ngFor="let p of node.parents" [fromX]="p.x" [toX]="node.x" [fromY]="p.y" [toY]="node.y"></g>
                </ng-container>

                <!-- all nodes on top of connectors -->
                <ng-container *ngFor="let node of allNodes.values()">
                    <g node [x]="node.x" [y]="node.y" [label]="node.label" [nodeType]="0"></g>
                </ng-container>
                
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


    // buckets of activations and spikes: horizontal ordering

    receivedSpikes: Map<number, SpikeUpdate> = new Map();
    allNodes: Map<number, Node> = new Map();

    columns: Array<Set<Node>> = [new Set()];




    receivedActivations: Map<number, ActivationUpdate> = new Map();


    constructor(private mockDataService: MockDataService, private sanitizer: DomSanitizer) {
        this.subscriptions = new Subscription();

        this.subscriptions.add(this.mockDataService.spikes.subscribe(spike => {
            this.receivedSpikes.set(spike.id, spike);

            const node = new Node(spike);
            this.allNodes.set(node.id, node);

            // find all node parents
            let parentInLastCol = false;
            for (const parentID of spike.parents) {
                const nodeID = Node.spikeID(parentID);
                const parent = this.allNodes.get(nodeID);
                if (parent) {
                    if (parent.column === this.columns.length - 1) {
                        parentInLastCol = true;
                    }
                    node.parents.push(parent);
                }
            }
            const lastColumnFull =  this.columns[this.columns.length - 1].size > 5;

            // create new column if necessary
            if (parentInLastCol || lastColumnFull) {
                this.columns.push(new Set());
            }

            // add new node to the last column and reposition nodes
            const lastColID = this.columns.length - 1;
            this.columns[lastColID].add(node);
            node.column = lastColID;

            // reposition all nodes in that column
            const ySpacing = 70;
            let y = - (this.columns[lastColID].size - 1) / 2 * ySpacing;
            for (const node of this.columns[lastColID].values()) {
                node.x = this.columns.length * this.nodeSpacingX;
                node.y = y;
                y += ySpacing;
            }


            /*const x = this.lastX;
            const y = (Math.random() * 2 - 1) * 200;
            const lastNode = this.nodes.length > 0 ? this.nodes[this.nodes.length - 1] : null;
            this.addNode(x, y, NodeType.SPIKE, 'spike', lastNode);
            this.containerPosX -= this.nodeSpacingX;
            this.lastX += this.nodeSpacingX;*/

        }));

        this.subscriptions.add(this.mockDataService.activations.subscribe(activation => {
            /*const x = this.lastX;
            const y = (Math.random() * 2 - 1) * 200;
            const lastNode = this.nodes.length > 0 ? this.nodes[this.nodes.length - 1] : null;
            this.addNode(x, y - 70, NodeType.ACTIVATION, 'activation 1', lastNode);
            this.addNode(x, y, NodeType.ACTIVATION, 'activation 2', lastNode);
            this.addNode(x, y + 70, NodeType.ACTIVATION, 'activation 3', lastNode);
            this.containerPosX -= this.nodeSpacingX;
            this.lastX += this.nodeSpacingX;*/
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
        const containerPosX = 800 - this.columns.length * this.nodeSpacingX;
        const transform = `translateX(900px) scale(${this.scale.toFixed(2)}) translateX(-900px) translateX(${containerPosX}px)`;
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
