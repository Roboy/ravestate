import { Component, ElementRef, OnDestroy, OnInit, ViewChild } from '@angular/core';
import { FormControl } from "@angular/forms";
import { SocketIOService } from "../../socketio/socketio.service";
import { Subscription } from "rxjs";
import { MessageFromUI } from "../../model/message-from-ui";
import { MessageToUI } from "../../model/message-to-ui";

@Component({
    selector: 'app-chat-window',
    template: `
        <div class="chat-header">
            <h3>Chat</h3>            
        </div>
        <div #msgArea class="message-area">
            <app-chat-message *ngFor="let m of messages" [message]="m">
            </app-chat-message>            
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
export class ChatWindowComponent implements OnInit, OnDestroy {

    @ViewChild('msgArea', {static: false}) msgArea: ElementRef<HTMLDivElement>;
    newMessage = new FormControl('');
    subs: Subscription = new Subscription();
    messages: Array<MessageFromUI | MessageToUI> = [
    ];

    constructor(private ioService: SocketIOService) {
    }

    ngOnInit(): void {
        this.subs.add(this.ioService.messagesToUI.subscribe(msg => {
            this.receive(msg); // message to ui
        }));
        this.subs.add(this.ioService.messagesFromUI.subscribe(msg => {
            this.receive(msg); // message from ui
        }));
    }

    ngOnDestroy(): void {
        this.subs.unsubscribe();
    }

    receive(msg: MessageFromUI | MessageToUI) {
        this.messages.push(msg);
        setTimeout(() => {
            // need timeout so browser calculates scrollHeight including the newly inserted element
            this.msgArea.nativeElement.scrollTop = this.msgArea.nativeElement.scrollHeight;
        }, 0);
    }

    send(msg: string) {
        if (!msg) {
            return;
        }
        this.newMessage.reset();
        this.ioService.sendMessage(msg);
    }

}
