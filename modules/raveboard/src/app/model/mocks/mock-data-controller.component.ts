import { Component } from '@angular/core';
import { SocketIOService } from "../../socketio/socketio.service";
import { MOCK_MESSAGES } from "./mock-messages";


@Component({
    selector: 'app-mock-data-controller',
    template: `
        <h3>Mock data</h3>
        <button (click)="sendNextMockMessage()">Send mock data</button>
        <button (click)="resetMockData()">Reset</button>
        <span class="mock-msg-counter">Sent: {{mockMessageCounter}} / {{mockMessageTotal}}</span>
        <br>
        <button (click)="sendActivation()">Send random activation</button>
        <button (click)="sendSpike()">Send random spike</button>
        <br>
        <h3>Mock messages</h3>
        <button (click)="chatSend()">Chat Send</button>
        <button (click)="chatReceive()">Chat Receive</button>
        <h3>Last mock message</h3>
        <div class="data-json">
            <pre>{{lastMockMessage ? (lastMockMessage | json) : '- none -'}}</pre>
        </div>
    `,
    styleUrls: ['./mock-data-controller.component.scss']
})
export class MockDataControllerComponent {

    private randomIDCounter: number = 100;
    private lastRandomSpikeID: number = -1;

    mockMessageCounter: number = 0;
    mockMessageTotal: number;
    lastMockMessage: any;

    constructor(private ioService: SocketIOService) {
        this.mockMessageTotal = MOCK_MESSAGES.length;
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

    chatSend() {
        this.ioService.sendMessage(this.randomMessage())
    }

    chatReceive() {
        this.ioService.messagesToUI.next({
            type: 'output',
            text: this.randomMessage()
        })
    }

    private randomMessage(): string {
        const words = 'lorem ipsum dolor sit amet consectetur adipiscing elit sed do eiusmod tempor'.split(' ');
        const n = Math.floor(Math.random() * 7) + 2;
        let msg = '';
        for (let i = 0; i < n; i++) {
            msg += words[Math.floor(Math.random() * words.length)] + ' ';
        }
        return msg;
    }
}
