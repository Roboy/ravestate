import { Injectable } from "@angular/core";
import { Subject } from "rxjs";
import * as io from "socket.io-client";

import { ActivationUpdate, MessageFromUI, SpikeUpdate } from "../model/model";


@Injectable({
    providedIn: 'root'
})
export class SocketIOService {

    // inputs: subscribe to receive messages
    activations: Subject<ActivationUpdate> = new Subject();
    spikes: Subject<SpikeUpdate> = new Subject();
    messagesToUI: Subject<MessageFromUI> = new Subject();

    // output: call emit() to send a message
    messagesFromUI: Subject<MessageFromUI> = new Subject();

    constructor() {
        const urlParams = new URLSearchParams(window.location.search);
        const sio_url = urlParams.get('rs-sio-url') || 'http://localhost:4242';

        console.log(`Connecting to socket.io URL: '${sio_url}'`);

        let socket = io.connect(sio_url);

        socket.on('spike', msg => {
            this.spikes.next(msg)
        });

        socket.on('activation', msg => {
            this.activations.next(msg)
        });

        socket.on('message', msg => {
            this.messagesToUI.next(msg);
        });

        this.messagesFromUI.subscribe(msg => { // no unsubscribe since this service lives forever
            socket.emit('message', msg);
        });
    }

}
