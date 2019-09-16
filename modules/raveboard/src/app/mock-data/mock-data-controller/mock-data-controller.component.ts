import { Component, OnDestroy } from '@angular/core';
import { MockDataService } from "../mock-data.service";
import { Subscription } from "rxjs";

@Component({
    selector: 'app-mock-data-controller',
    template: `
        Mock Data Controller
        <br>
        <button (click)="mockDataService.sendNextMockMessage()">Send mock message</button>
        <button (click)="mockDataService.resetMockData()">Reset mock messages</button>
        <br>
        <button (click)="mockDataService.sendActivation()">Send activation</button>
        <button (click)="mockDataService.sendSpike()">Send spike</button>
        <br>
        <br>
        Recent data
        <br>
        <div class="data-json">
            <pre>{{lastData ? (lastData | json) : '- none -'}}</pre>
        </div>
    `,
    styleUrls: ['./mock-data-controller.component.scss']
})
export class MockDataControllerComponent implements OnDestroy {

    lastData: any;
    dataSub: Subscription;

    i = 0;
    getI() {
        return this.i++;
    }

    constructor(public mockDataService: MockDataService) {
        this.dataSub = mockDataService.dataStream.subscribe(data => {
            this.lastData = data;
        });
    }

    ngOnDestroy(): void {
        this.dataSub.unsubscribe();
    }
}
