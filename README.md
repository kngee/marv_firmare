### EMK 310: Race Day Firmware 2025

![The Whip](https://media3.giphy.com/media/v1.Y2lkPTc5MGI3NjExNDBibzRwaGs5ZHh2OWZ4bmV4Mm8wOGpsM3NkZnB0amlnaHFjNTlwaSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/fnTytq0RJGjIc4j0q5/giphy.gif)

- [X] Fix state rotation
- [X] SSD for racing
- [X] Fix logic for black/green
- [X] Program memory for calibration
- [Undecided] Integral control?
- [Ongoing] General code review

##### _22 May 2025_

- [ ] Test Calibration Update for Red

we want the calibration sequence for red to no longer use the starting grid, yesterday we found the problem of any instance of red on the line being pulled up to white. The new approach will allow us to place each sensor on the line before calibration.

Possible update: Can you write the calibration sequence so the sensors keep track of the most likely value while it performs some motion over the line.

- [ ] Verify relative ordering for Tests

Yesterday we found that it may not be true that red is consistently below white, therefore we need to make sure that we make these comparisons in the right order

- [ ] Drop the number of sample to improve speed

Readings are consistent after the first ADC conversion and so, taking 2 good samples will more than likely be enough for good performance.
