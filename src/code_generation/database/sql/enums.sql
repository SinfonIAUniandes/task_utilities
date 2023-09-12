/*Creación del enum del resultado de la tarea*/
CREATE TYPE llm_model AS ENUM {
  'GPT3.5',
  'LLAMA2'
};

ALTER TABLE "pepper_tests"
ADD COLUMN model_name llm_model NOT NULL
DEFAULT 'GPT3.5';

CREATE TYPE task_execution_result_enum AS ENUM (
  'NOT_EXECUTED',
  'PASSED_AUTOMATIC_EXECUTION',
  'EXECUTED_BUT_FAILED',
  'PARTIALLY_COMPLETED',
  'SUCCESFULLY_COMPLETED'
);

ALTER TABLE "pepper_tests"
ADD COLUMN task_execution_result task_execution_result_enum NOT NULL
DEFAULT 'NOT_EXECUTED';

CREATE TYPE prompting_type_enum AS ENUM (
  'LONG_STRING',
  'CHAINING'
);

ALTER TABLE "pepper_tests"
ADD COLUMN prompting_type prompting_type_enum NOT NULL
DEFAULT 'LONG_STRING';

CREATE TYPE task_category_enum AS ENUM (
  'GSPR1',
  'GSPR2',
  'GSPR3',
  'EGSPR1',
  'EGSPR2',
  'EGSPR3',
  'EGSPR4',
  'EGSPR5',
  'EGSPR6',
  'EGSPR7',
  'EGSPR8',
  'EGSPR9'
)

ALTER TABLE "pepper_tests"
ADD COLUMN task_category task_category_enum NOT NULL
DEFAULT 'GSPR1';

/*Operación de peligro*/

DROP TYPE IF EXISTS task_execution_result_enum CASCADE;
DROP TYPE IF EXISTS llm_model CASCADE;
