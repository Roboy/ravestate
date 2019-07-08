import { Component, OnDestroy } from '@angular/core';
import { MockDataService } from "../mock-data.service";
import { Subscription } from "rxjs";

@Component({
    selector: 'app-mock-data-controller',
    template: `
        Mock Data Controller
        <br>
        <button (click)="mockDataService.sendActivations()">Send activations</button>
        <button (click)="mockDataService.sendSpike()">Send spike</button>
        <br><br>
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

    constructor(public mockDataService: MockDataService) {
        this.dataSub = mockDataService.dataStream.subscribe(data => {
            this.lastData = data;
            console.log(JSON.stringify(data, null, 2))
        });
    }

    ngOnDestroy(): void {
        this.dataSub.unsubscribe();
    }
}
