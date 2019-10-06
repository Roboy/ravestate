import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';
import { ReactiveFormsModule } from '@angular/forms';

import { AppComponent } from './app.component';
import { NodeComponent } from './svg/elements/node.component';
import { ConnectorComponent } from './svg/elements/connector.component';
import { MockDataControllerComponent } from './model/mocks/mock-data-controller.component';
import { ActivationSpikeGraphComponent } from './svg/activation-spike-graph/activation-spike-graph.component';
import { ChatWindowComponent } from './chat/chat-window/chat-window.component';
import { ChatMessageComponent } from './chat/chat-message/chat-message.component';

@NgModule({
    declarations: [
        AppComponent,
        NodeComponent,
        ConnectorComponent,
        MockDataControllerComponent,
        ActivationSpikeGraphComponent,
        ChatWindowComponent,
        ChatMessageComponent
    ],
    imports: [
        BrowserModule,
        ReactiveFormsModule
    ],
    providers: [],
    bootstrap: [AppComponent]
})
export class AppModule {
}
