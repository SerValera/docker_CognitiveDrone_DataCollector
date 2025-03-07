# Data Organization Structure

The data should be organized in the following structure:

## Category
- **Type folder**
- JSON file with the same name as the **Type folder**

### Example:
```
dataset_to_record
├──Reasoning/
│   ├── Digits/
│       ├── one.dae
│       ├── two.dae
│       ├── ...
│       ├── ten.dae
│   ├── Symbols/
│       ├── A.dae
│       ├── B.dae
│       ├── ...
│       ├── Z.dae
│   ├── Digits.json
│   ├── Symbols.json
├──Symbol Understanding/
...
```

Each **Type folder** contains *.dae files, and the corresponding JSON file provides metadata.

Make sure that the items of option parameter of the *.json file have same names from **Type folder** folder.

Example: 

    "options": [
        "ten.dae",
        "eight.dae",
        "one.dae"
    ]




[Next: Go to Start Recording](3_recording.md)

[Back: Go back to docker preporation](0_docker.md)

[Back: Go to Start Simulation](1_sessions.md)