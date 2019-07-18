
## Example for Ravestate UI event message sequence

```json
{
    "type": "spike",
    "signal": "rawio:in:changed",
    "id": 0,
    "parents": [],
}
```
```json
{
    "type": "activation",
    "id": 0,
    "state": "wildtalk",
    "specificity": 0.2,
    "parents": [],
    "constraints": [{
        "rawio:in:changed": 0
    }]
}
```

![Pic2](1.png)

![Pic2](2.png)

![Pic3](3.png)

![Pic4](4.png)

![Pic5](5.png)

![Pic6](6.png)

![Pic7](7.png)
