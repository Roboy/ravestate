import { Component, OnInit } from '@angular/core';

@Component({
    selector: 'app-chat-window',
    template: `
        <div class="chat-header">
            <h3>Chat</h3>            
        </div>
        <div class="message-area">
            
        </div>
        <div class="new-message-area">
            <input class="new-message-input" type="text" #msg placeholder="type here ...">
            <button class="new-message-send">Send</button>
        </div>
    `,
    styleUrls: ['./chat-window.component.scss']
})
export class ChatWindowComponent implements OnInit {

    constructor() {
    }

    ngOnInit() {
    }

}
