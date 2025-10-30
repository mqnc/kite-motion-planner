# kite-motion-planner
Proof-of-Concept Motion Planner for Robot Arms

## About

This project is an experimental successor to the [Gestalt Motion Planner](https://github.com/mqnc/gestalt-motion-planner). It is centered around a concept that allows for very efficient motion validation that can theoretically guarantee collision-free motion (in contrast to the usual sampling or linear continuous collision detection approaches).

## Project Status

Despite having been developed to be used in production later on, this planner is still in experimental state. The author is no longer working at Gestalt Automation GmbH where the project was initially started and is now maintaining it as a hobby.

## License

This project is released under the [PolyForm Noncommercial License 1.0.0](https://polyformproject.org/licenses/noncommercial/1.0.0/). For a commercial license contact [Gestalt Automation GmbH](https://www.gestalt-automation.com/en/contact).

## Hierarchical Sweeping

The core concept is somewhat similar to what is outlined in [this paper](https://www.cs.unc.edu/techreports/03-038.pdf) but utilizes some tweaks that make it slightly more conservative but much more efficient.

