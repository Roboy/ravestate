import { Component } from '@angular/core';
import { SocketIOService } from "../../socketio/socketio.service";
import { MOCK_MESSAGES } from "./mock-messages";


@Component({
    selector: 'app-mock-data-controller',
    template: `
        Mock Data Controller
        <br>
        <button (click)="sendNextMockMessage()">Send mock message</button>
        <button (click)="resetMockData()">Reset mock messages</button>
        <br>
        <button (click)="sendActivation()">Send activation</button>
        <button (click)="sendSpike()">Send spike</button>
        <br>
        <br>
        Recent data
        <br>
        <div class="data-json">
            <pre>{{lastMockMessage ? (lastMockMessage | json) : '- none -'}}</pre>
        </div>
    `,
    styleUrls: ['./mock-data-controller.component.scss']
})
export class MockDataControllerComponent {

    private randomIDCounter: number = 100;
    private mockMessageCounter: number = 0;

    private lastRandomSpikeID: number = -1;

    lastMockMessage: any;

    constructor(private ioService: SocketIOService) {
    }

    sendNextMockMessage() {
        if (this.mockMessageCounter < MOCK_MESSAGES.length) {
            this.lastMockMessage = MOCK_MESSAGES[this.mockMessageCounter];
            if (this.lastMockMessage.type === 'activation') {
                this.ioService.activations.next(this.lastMockMessage);
            } else {
                this.ioService.spikes.next(this.lastMockMessage);
            }
            this.mockMessageCounter++;
        } else {
            this.sendSpike();
        }
    }

    resetMockData() {
        this.mockMessageCounter = 0;
    }

    sendActivation() {
        this.lastMockMessage = {
            type: 'activation',
            id: this.randomIDCounter,
            state: 'random state',
            specificity: Math.floor(Math.random() * 100) / 100,
            status: 'ready',
            spikes: [{
                'random spike 1': this.randomIDCounter - 1,
                'random spike 2': this.randomIDCounter - 2,
                'random spike 4': this.randomIDCounter - 4,
                'another random spike': this.lastRandomSpikeID,
            }]
        };
        this.randomIDCounter++;
        this.ioService.activations.next(this.lastMockMessage);
    }

    sendSpike() {
        this.lastMockMessage = {
            type: 'spike',
            id: this.randomIDCounter,
            signal: 'random spike',
            parents: [this.randomIDCounter - 8, this.randomIDCounter - 14]
        };
        this.lastRandomSpikeID = this.randomIDCounter;
        this.randomIDCounter++;
        this.ioService.spikes.next(this.lastMockMessage);
    }
}
