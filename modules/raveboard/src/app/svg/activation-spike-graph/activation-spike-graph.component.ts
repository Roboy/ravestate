import { Component, ElementRef, OnDestroy, ViewChild } from '@angular/core';
import { Subscription } from "rxjs";

import { NodeType } from "../elements/node.component";
import { DomSanitizer } from "@angular/platform-browser";
import { SocketIOService } from "../../socketio/socketio.service";
import { SpikeUpdate } from "../../model/spike-update";
import { ActivationUpdate } from "../../model/activation-update";

export class NodeData {
    id: number;
    column: number;

    element: SpikeUpdate | ActivationUpdate;
    parents: Array<NodeData>;

    x: number;
    y: number;
    label: string;
    visible: boolean;
    transparent: boolean;
    nodeType: NodeType;

    constructor(element: SpikeUpdate | ActivationUpdate) {
        this.element = element;
        this.visible = true;
        this.transparent = false;
        this.parents = [];
        if (element.type === 'activation') {
            this.label = element.state; // `${element.state} [${element.id}]`;
            this.id = NodeData.activationID(element.id);
            this.nodeType = NodeType.ACTIVATION;
        } else {
            this.label = element.signal; // `${element.signal} [${element.id}]`;
            this.id = NodeData.spikeID(element.id);
            this.nodeType = NodeType.SPIKE;
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
        <div #wrapper style="overflow-x: scroll; width: 100%; height: 100%;">
            <svg #svg>
                <g [style.transform]="getTransform()">
                    
                    <!-- connectors for every node -->
                    <ng-container *ngFor="let node of allNodes.values()">
                        <ng-container *ngIf="node.visible">
                            <g connector *ngFor="let p of node.parents" [fromX]="p.x" [toX]="node.x" [fromY]="p.y"
                               [toY]="node.y"
                               [style.opacity]="hoveredNode && hoveredNode != node ? .1 : 1"></g>
                        </ng-container>
                    </ng-container>
    
                    <!-- all nodes on top of connectors -->
                    <ng-container *ngFor="let node of allNodes.values()">
                        <g node *ngIf="node.visible"
                           (mouseenter)="hoverStart(node)" (mouseleave)="hoverEnd()"
                           [x]="node.x" [y]="node.y" [label]="node.label"
                           [nodeType]="node.nodeType" [nodeStatus]="node.element.status"
                           [style.opacity]="node.transparent ? .2 : 1"></g>
                    </ng-container>
    
                </g>
            </svg>
        </div>
        <div class="controls">
            <button (click)="scaleDown()" class="round">-</button>
            <span class="percentage-label">{{(scale * 100).toFixed(0)}}%</span>
            <button (click)="scaleUp()" class="round">+</button>
            <button (click)="resetScale()" [disabled]="scale == 1">100%</button>
            <span class="separator">|</span>
            <button (click)="clear()">Clear Graph</button>
        </div>
    `,
    styleUrls: ['./activation-spike-graph.component.scss']
})
export class ActivationSpikeGraphComponent implements OnDestroy {

    private subscriptions: Subscription;

    // distance between two nodes (x-axis)
    nodeSpacingX = 180;
    nodeSpacingY = 100;
    maxNodesPerColumn = 5;

    scale = 1;

    @ViewChild('wrapper', {static: true} ) wrapper: ElementRef<HTMLDivElement>;
    @ViewChild('svg', {static: true} ) svg: ElementRef<SVGElement>;

    allNodes: Map<number, NodeData> = new Map();
    columns: Array<Set<NodeData>> = [new Set()];

    hoveredNode: NodeData;

    constructor(private socketIoService: SocketIOService, private sanitizer: DomSanitizer) {

        this.subscriptions = new Subscription();

        this.subscriptions.add(socketIoService.spikes.subscribe(spike => {

            const node = new NodeData(spike);
            this.allNodes.set(node.id, node);

            // find all spike parents
            let parentInLastCol = false;
            for (const parentID of spike.parents) {
                const nodeID = NodeData.spikeID(parentID);
                const parent = this.allNodes.get(nodeID);
                if (parent) {
                    if (parent.column === this.columns.length - 1) {
                        parentInLastCol = true;
                    }
                    node.parents.push(parent);
                }
            }
            const lastColumnFull = this.columns[this.columns.length - 1].size >= this.maxNodesPerColumn;

            // create new column if necessary
            if (parentInLastCol || lastColumnFull) {
                this.columns.push(new Set());
                this.scheduleScrollSnapCheck();
            }

            // add new node to the last column and reposition nodes
            const lastColID = this.columns.length - 1;
            this.columns[lastColID].add(node);
            node.column = lastColID;

            this.repositionColumn(lastColID);  // reposition all nodes in that column

        }));

        this.subscriptions.add(socketIoService.activations.subscribe(activation => {

            // create a new node for incoming activation
            const newNode = new NodeData(activation);

            // find all activation parents, determine lowest allowed column id
            let highestParentColumn = -1;
            for (const spikeMap of activation.spikes) {
                for (const spikeID of Object.values(spikeMap)) {
                    const nodeID = NodeData.spikeID(spikeID);
                    const parent = this.allNodes.get(nodeID);
                    if (parent) {
                        highestParentColumn = Math.max(highestParentColumn, parent.column);
                        newNode.parents.push(parent);
                    }
                }
            }
            // activations are only visible if they have parent spikes
            newNode.visible = newNode.parents.length > 0;

            // check if node with same activation id already exists,
            const prevNode = this.allNodes.get(newNode.id);

            let targetColumnID;
            if (prevNode && prevNode.column > highestParentColumn) {
                // replace prev with new node, in the same column
                targetColumnID = prevNode.column;

                // create new set to keep order, replace prev node
                const newColSet: Set<NodeData> = new Set();
                for (const colNode of this.columns[targetColumnID]) {
                    newColSet.add(colNode != prevNode ? colNode : newNode);
                }
                this.columns[targetColumnID] = newColSet;

            } else {
                // new must go to new column
                if (prevNode) { // delete prev
                    this.columns[prevNode.column].delete(prevNode);
                    this.repositionColumn(prevNode.column);  // reposition column where prev node was deleted
                }

                // need to insert new into last column
                if (highestParentColumn >= this.columns.length - 1
                    || this.columns[this.columns.length - 1].size >= this.maxNodesPerColumn) {
                    // last column not ok (full or has parents)
                    this.columns.push(new Set());
                    this.scheduleScrollSnapCheck();
                }
                targetColumnID = this.columns.length - 1;

                this.columns[targetColumnID].add(newNode);

            }

            newNode.column = targetColumnID;
            this.repositionColumn(targetColumnID);  // reposition column
            this.allNodes.set(newNode.id, newNode); // will also replace prevNode if it was there
        }));
    }

    private repositionColumn(targetColumnID: number) {
        let visibleNodes = 0;
        for (const node of this.columns[targetColumnID].values()) {
            if (node.visible) {
                visibleNodes++;
            }
        }

        // reposition column
        let y = -(visibleNodes - 1) / 2 * this.nodeSpacingY;
        for (const node of this.columns[targetColumnID].values()) {
            if (!node.visible) {
                continue;
            }
            node.x = targetColumnID * this.nodeSpacingX + 85;
            node.y = y;
            y += this.nodeSpacingY;
        }

    }

    ngOnDestroy(): void {
        this.subscriptions.unsubscribe();
    }

    clear() {
        this.allNodes.clear();
        this.columns = [new Set()];
    }

    scaleUp() {
        if (this.scale < 4) {
            this.scaleBy(1.1);
        }
    }

    scaleDown() {
        if (this.scale >= 0.3) {
            this.scaleBy(1 / 1.1);
        }
    }

    resetScale() {
        this.scaleBy(1 / this.scale);
    }

    scaleBy(factor: number) {
        this.scale *= factor;
        const e = this.wrapper.nativeElement;
        const dx = (e.scrollLeft + e.clientWidth / 2) * (factor - 1);
        const target = e.scrollLeft + dx;
        if (dx > 0) {
            this.svg.nativeElement.style.width = this.columns.length * this.nodeSpacingX * this.scale + 'px';
            e.scrollLeft = target;
        } else {
            e.scrollLeft = target;
            this.svg.nativeElement.style.width = this.columns.length * this.nodeSpacingX * this.scale + 'px';
        }
    }

    getTransform() {
        const transform = `scale(${this.scale.toFixed(6)})`;
        return this.sanitizer.bypassSecurityTrustStyle(transform);
    }

    hoverStart(hoveredNode: NodeData) {
        for (const node of this.allNodes.values()) {
            node.transparent = true;
        }
        hoveredNode.transparent = false;
        for (const parent of hoveredNode.parents) {
            parent.transparent = false;
        }


        this.hoveredNode = hoveredNode;
    }

    hoverEnd() {
        for (const node of this.allNodes.values()) {
            node.transparent = false;
        }
        this.hoveredNode = null;
    }

    scheduleScrollSnapCheck() {
        const e = this.wrapper.nativeElement;
        const snap = e.scrollLeft >= (e.scrollWidth - e.clientWidth);
        this.svg.nativeElement.style.width = this.columns.length * this.nodeSpacingX * this.scale + 'px';
        if (snap) {
            e.scrollLeft = e.scrollWidth;
        }
    }

}
