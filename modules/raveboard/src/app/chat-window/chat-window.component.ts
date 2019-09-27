import { Component } from '@angular/core';
import { FormControl } from "@angular/forms";
import { SocketIOService } from "../socketio/socketio.service";
import { Subscription } from "rxjs";

@Component({
    selector: 'app-chat-window',
    template: `
        <div class="chat-header">
            <h3>Chat</h3>            
        </div>
        <div class="message-area">
            <div *ngFor="let m of messages">
                {{m}}
            </div>            
        </div>
        <div class="new-message-area">
            <input class="new-message-input" type="text" placeholder="type here ..."
                   [formControl]="newMessage"                
                   (keyup.enter)="send(newMessage.value)">
            <button class="new-message-send" [disabled]="!newMessage.value" (click)="send(newMessage.value)">Send</button>
        </div>
    `,
    styleUrls: ['./chat-window.component.scss']
})
export class ChatWindowComponent {

    newMessage = new FormControl('');
    subs: Subscription = new Subscription();

    messages: Array<string> = [];

    constructor(private ioService: SocketIOService) {
        this.subs.add(ioService.messagesToUI.subscribe(msg => {
            // message to ui
            this.messages.push('received: ' + msg.text);
        }));

        this.subs.add(ioService.messagesFromUI.subscribe(msg => {
            // message from ui
            this.messages.push('sent: ' + msg.text);
        }));
    }

    send(msg: string) {
        if (!msg) {
            return;
        }
        this.newMessage.reset();
        this.ioService.sendMessage(msg);
    }

}
