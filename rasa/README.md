# Rasa NLU/NLG

## Add new data & features

### New intent

1. Add new intent in `./data/nlu_task.yml` file.

```
 - intent: <intent Name>
    examples: |
      - <example 1>
      - <example 2>
      - <example 3>
```

2. add intent's name below `intents` in `./domain.yml` file.
3. add rules for specific action after intent is detected in `./data/rules.yml` file. (option)

```
- rule: <rule's name>
    steps:
      - intent: <intent>
      - action: <action>
```

### New custom actions

1. Add new action in `./actions/actions.py` file.

## How to train and test

- You can train new model by this command.

```
rasa train
```

- This command is testing the model. Open debug mode by add suffix `-debug`.

```shell
rasa shell
```
