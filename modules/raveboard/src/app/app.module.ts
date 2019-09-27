import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';
import { ReactiveFormsModule } from '@angular/forms';

import { AppComponent } from './app.component';
import { NodeComponent } from './svg/elements/node.component';
import { ConnectorComponent } from './svg/elements/connector.component';
import { MockDataControllerComponent } from './model/mocks/mock-data-controller.component';
import { ActivationSpikeGraphComponent } from './svg/activation-spike-graph/activation-spike-graph.component';
import { ChatWindowComponent } from './chat-window/chat-window.component';

@NgModule({
    declarations: [
        AppComponent,
        NodeComponent,
        ConnectorComponent,
        MockDataControllerComponent,
        ActivationSpikeGraphComponent,
        ChatWindowComponent
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
