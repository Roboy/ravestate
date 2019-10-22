import { Component, Input } from '@angular/core';

import { MessageFromUI } from "../../model/message-from-ui";
import { MessageToUI } from "../../model/message-to-ui";

@Component({
    selector: 'app-chat-message',
    template: `
        <div [ngClass]="message.type + '-message-wrapper'">
            <div *ngIf="message" [ngClass]="message.type + '-message'">{{message.text}}</div>            
        </div>
    `,
    styleUrls: ['./chat-message.component.scss']
})
export class ChatMessageComponent {
    @Input() message: MessageFromUI | MessageToUI;
}
